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

void RL_Compute(const TrackModel *m, float speedFrac, const RecoverState *rcv,
                RacingLine *out)
{
    uint8_t la, segsNeeded;
    float   hn, hf, a, b, turn;
    float   bias, target, half, usable, conf;
    bool    straight;
    float   lo, hi, yNear, span;
    uint8_t j;

    out->chicane  = false;
    out->clamped  = false;
    out->probe    = 0.0f;
    out->wEntry   = 0.0f;
    out->wApex    = 0.0f;
    out->wExit    = 0.0f;
    out->conf     = LINE_CONF_MIN;
    out->straight = false;
    out->chorded  = false;

    if (!m->haveTrack)
    {
        out->targetX = CAM_CENTER_X;
        out->laRow   = 0u;
        out->bias    = s_bias;
        return;
    }

    /*
     * How well did the camera describe the curve? One vector per edge is a chord:
     * accurate at its two ends, but sagging to the inside of the bend everywhere in
     * between, and carrying no curvature at all. Several short vectors describe the
     * same curve properly.
     *
     * All of which is an argument about curves. Laid along a straight the chord is
     * not an approximation of anything - it is the line - so the two vectors a real
     * Pixy2 hands back from a straight road describe that road completely. On real
     * hardware that is most of a lap, so how many vectors are needed is asked of the
     * geometry rather than fixed. See LINE_CONF_SEGS_STRAIGHT for why the test is
     * written the way it is, and in particular why curv cannot be what it rests on.
     */
    straight = (fabsf(m->headFar) < LINE_CONF_STRAIGHT_HEAD) &&
               (fabsf(m->curv) < LINE_CONF_STRAIGHT_CURV) && m->bothEdges &&
               (m->nValid >= (uint8_t)LINE_CONF_STRAIGHT_ROWS);

    segsNeeded = straight ? (uint8_t)LINE_CONF_SEGS_STRAIGHT
                          : (uint8_t)LINE_CONF_SEGS_FULL;

    if (m->segCount >= segsNeeded)
    {
        conf = 1.0f;
    }
    else
    {
        float t = (float)m->segCount / (float)segsNeeded;

        conf = LINE_CONF_MIN + ((1.0f - LINE_CONF_MIN) * clampf(t, 0.0f, 1.0f));
    }

    out->conf     = conf;
    out->straight = straight;

    /*
     * The specific case the speed planner has to be told about, which is narrower
     * than "confidence is not full". Curvature is a difference between two headings,
     * so measuring it takes two vectors along the same edge - three points. One
     * vector per edge gives two points and a slope, and a slope is a straight: curv
     * comes out exactly zero no matter how hard the road is bending. A corner
     * described like that does not read as a mild corner, it reads as no corner, and
     * every cue built on curvature agrees with it.
     *
     * A corner with a bend measured anywhere in it is a corner the cues can see, and
     * it is left alone - taxing it would cost the lap everywhere and buy nothing.
     * Note this is not a count: two vectors along one edge measure a bend, while two
     * vectors one per edge do not, and only track.c knows which of those it had.
     */
    out->chorded = (!straight) && (!m->canCurve);

    /* ---------------------------------------------------------------
     * 1. How far ahead to aim.
     *
     * Normally this is set by speed alone. But when the curve has been chorded, the
     * only two points guaranteed to be on the real line are the ends of that chord -
     * so the aim point is pushed outward, onto the far end, where the error goes back
     * to zero. Aiming at a middle row would aim at the sag, which is inside the
     * corner, and the car would turn in early.
     * -------------------------------------------------------------*/
    {
        float f = clampf(speedFrac, 0.0f, 1.0f);
        float r = (float)LINE_LA_ROW_MIN + (f * (float)(LINE_LA_ROW_MAX - LINE_LA_ROW_MIN));

        r += (1.0f - conf) * LINE_LA_LOWCONF_BOOST;

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

    /* The chord already leans toward the inside of the bend; diving for an apex on
     * top of that is how the car ends up cutting the corner. So the racing line is
     * scaled back by the same confidence. */
    bias *= conf;

    /*
     * Probing for a line that left the frame. Everything the bias was built
     * from - which phase of the corner this is, how hard it bends - came out
     * of a headFar that is really just the slope of the single edge still in
     * view. Committing to a racing line on that is leaning on a corner nobody
     * measured, so it goes while the probe runs. This is the straightening
     * half of the recovery, and it happens before the lean below.
     */
    if (rcv->active)
    {
        bias *= 1.0f - clampf(RCV_BIAS_CUT * rcv->nudge, 0.0f, 1.0f);
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

    /*
     * ...and the lean. Easing the aim point toward the side the camera has
     * lost yaws the car a few degrees that way, and yaw is what brings a line
     * back into a frame this narrow - the car barely has to move.
     *
     * This is the one place the aim point is allowed toward an edge that was
     * inferred rather than measured, which the guard immediately above spends
     * its time preventing. The difference is what it is for: the guard stops
     * the car spending margin on a racing line through a corner it cannot see,
     * while this spends a much smaller amount of it to get the measurement
     * back, gives up after RCV_MAX_FRAMES if that does not work, and stops the
     * instant the line reappears. It also stays underneath the safety check
     * below, which is what stops it walking the car into the edge it is
     * guessing at.
     */
    if (rcv->active)
    {
        float lean = rcv->lean * RCV_NUDGE_FRAC;

        target += lean * usable;
        out->probe = lean;
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
