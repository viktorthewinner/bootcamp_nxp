#include "racing_line.h"
#include "race_config.h"
#include <math.h>

static float s_bias; /* smoothed, so the aim point sweeps instead of jumping */

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static float signf(float v)
{
    if (v > 0.0f)
    {
        return 1.0f;
    }
    if (v < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

/* t^PATH_SHAPE_EXP without calling powf, which is expensive on a Cortex-M33. */
static float path_shape(float t)
{
    /* 1.5 -> t * sqrt(t), one VSQRT instruction */
    return t * sqrtf(t);
}

void RL_Init(void)
{
    s_bias = 0.0f;
}

void RL_Compute(const TrackModel *m, float speedFrac, RacingLine *out)
{
    uint8_t la;
    float   hn, hf, a, b, turn;
    float   bias, target, half, usable;
    float   lo, hi, yNear, span;
    uint8_t j;

    out->chicane = false;
    out->clamped = false;
    out->wEntry  = 0.0f;
    out->wApex   = 0.0f;
    out->wExit   = 0.0f;

    if (!m->haveTrack)
    {
        out->targetX = CAM_CENTER_X;
        out->laRow   = 0u;
        out->bias    = s_bias;
        return;
    }

    /* ---------------------------------------------------------------
     * 1. How far ahead to aim.
     * -------------------------------------------------------------*/
    {
        float f = clampf(speedFrac, 0.0f, 1.0f);
        float r = (float)LINE_LA_ROW_MIN + (f * (float)(LINE_LA_ROW_MAX - LINE_LA_ROW_MIN));

        la = (uint8_t)(r + 0.5f);
        if (la > m->topRow)
        {
            la = m->topRow;
        }
        if (la < 1u)
        {
            la = (m->topRow >= 1u) ? 1u : 0u;
        }
    }

    /* ---------------------------------------------------------------
     * 2. Where the car is in the corner.
     * -------------------------------------------------------------*/
    hn = m->headNear;
    hf = m->headFar;

    a = clampf(fabsf(hn) / LINE_HEAD_REF, 0.0f, 1.0f); /* how much we are in it   */
    b = clampf(fabsf(hf) / LINE_HEAD_REF, 0.0f, 1.0f); /* how much is still ahead */

    turn = (fabsf(hf) > fabsf(hn)) ? signf(hf) : signf(hn);

    /* Entry uses its own, more sensitive scale - see LINE_ENTRY_HEAD_REF. */
    {
        float bEntry = clampf(fabsf(hf) / LINE_ENTRY_HEAD_REF, 0.0f, 1.0f);

        out->wEntry = bEntry * (1.0f - a);
    }
    out->wApex = a * b;
    out->wExit = a * (1.0f - b);

    /* + drives the aim point toward the right edge. For a right hand corner
     * (turn = +1) the apex is on the right and the outside is on the left, so apex
     * pushes positive and entry and exit push negative. Mirrored automatically for
     * a left hand corner because turn flips sign. */
    bias = turn * ((LINE_APEX_BIAS * out->wApex) - (LINE_ENTRY_BIAS * out->wEntry) -
                   (LINE_EXIT_BIAS * out->wExit));

    /* ---------------------------------------------------------------
     * 3. Small chicanes are not corners. Drive straight through them.
     * A chicane is the road bending one way close up and the other way further
     * out, with neither bend big enough to be worth a steering input.
     * -------------------------------------------------------------*/
    if (((hn * hf) < 0.0f) && (fabsf(hn) < CHICANE_MAX_HEAD) && (fabsf(hf) < CHICANE_MAX_HEAD))
    {
        out->chicane = true;
        bias         = 0.0f;
    }

    s_bias += LINE_BIAS_ALPHA * (bias - s_bias);
    s_bias = clampf(s_bias, -1.0f, 1.0f);

    /* ---------------------------------------------------------------
     * 4. Turn the bias into a column.
     * usable is the half width minus the keep-out band, so a bias of exactly 1
     * lands on the edge of the margin and never on the black line itself.
     * -------------------------------------------------------------*/
    half   = 0.5f * m->width[la];
    usable = half - m->margin[la];
    if (usable < 0.0f)
    {
        usable = 0.0f;
    }

    if (out->chicane)
    {
        /* Aim at the average centre over everything in view: that is the straight
         * line through the S, which is exactly what should be driven. */
        float sum = 0.0f;
        float cnt = 0.0f;

        for (j = 0u; j <= la; j++)
        {
            if (m->valid[j])
            {
                sum += m->center[j];
                cnt += 1.0f;
            }
        }
        target = (cnt > 0.0f) ? (sum / cnt) : m->center[la];
    }
    else
    {
        target = m->center[la];
    }

    target += s_bias * usable;

    /*
     * Moving to the outside before turn in is the whole point of the entry phase, so
     * it is allowed to steer away from the corner - but only toward an edge the
     * camera can actually see. When the outside line has been inferred rather than
     * measured, its position is a guess from the width model, and driving toward a
     * guess is how a car ends up on the wrong side of it. In that case the aim point
     * is not allowed past the car's own axis.
     */
    if ((out->wEntry > out->wApex) && (out->wEntry > out->wExit) && (turn != 0.0f))
    {
        bool outsideSeen = (turn > 0.0f) ? m->sawL[la] : m->sawR[la];

        if (!outsideSeen)
        {
            if ((turn > 0.0f) && (target < CAM_CENTER_X))
            {
                target = CAM_CENTER_X;
            }
            else if ((turn < 0.0f) && (target > CAM_CENTER_X))
            {
                target = CAM_CENTER_X;
            }
            else
            {
                /* already aiming into the corner, nothing to hold back */
            }
        }
    }

    /* ---------------------------------------------------------------
     * 5. The hard safety rule.
     *
     * The car is assumed to sweep from where it is now to the aim point, following
     * roughly px(t) = CAM_CENTER_X + shape(t) * (target - CAM_CENTER_X). Every
     * sample row in between turns into a pair of bounds on target:
     *
     *      xl + margin <= px(t) <= xr - margin
     *
     * Solving each of those for target and keeping the tightest pair gives the exact
     * window of aim points that keeps the whole path inside the black lines. This
     * also quietly overrides an over-enthusiastic apex, and cancels a chicane that
     * turned out to be too tight to ignore.
     * -------------------------------------------------------------*/
    lo    = -1.0e9f;
    hi    = 1.0e9f;
    yNear = m->y[0];
    span  = yNear - m->y[la];

    if (span > 0.5f)
    {
        for (j = 1u; j <= la; j++)
        {
            float t, s, lmin, rmax, loJ, hiJ;

            if (!m->valid[j])
            {
                continue;
            }

            t = (yNear - m->y[j]) / span;
            s = path_shape(clampf(t, 0.0f, 1.0f));
            if (s < 0.05f)
            {
                continue; /* too close to the car for the aim point to move it */
            }

            lmin = m->xl[j] + m->margin[j];
            rmax = m->xr[j] - m->margin[j];

            loJ = CAM_CENTER_X + ((lmin - CAM_CENTER_X) / s);
            hiJ = CAM_CENTER_X + ((rmax - CAM_CENTER_X) / s);

            if (loJ > lo)
            {
                lo = loJ;
            }
            if (hiJ < hi)
            {
                hi = hiJ;
            }
        }
    }

    if (lo > hi)
    {
        /* No aim point satisfies every row at once - the corridor pinches somewhere.
         * Split the difference, which is the least bad compromise, and let the speed
         * planner deal with it by slowing down. */
        target       = 0.5f * (lo + hi);
        out->clamped = true;
    }
    else
    {
        float before = target;

        target = clampf(target, lo, hi);
        if (fabsf(target - before) > 0.25f)
        {
            out->clamped = true;
        }
    }

    target = clampf(target, 0.0f, (float)(PIXY_LINE_W - 1));

    out->targetX = target;
    out->laRow   = la;
    out->bias    = s_bias;
}
