#include "track.h"
#include <math.h>

#define TRK_MAX_SEGS 16

typedef struct
{
    float xTail, yTail; /* nearest end  */
    float xHead, yHead; /* farthest end */
    float dxdy;         /* image slope, x per y                     */
    float len;          /* length in pixels                         */
    float span;         /* rows covered, yTail - yHead              */
} Seg;

static const uint8_t s_rowY[TRK_ROWS] = TRK_ROW_Y_INIT;

/* Corridor width as a straight line in image row: width(y) = s_wA * y + s_wB.
 * One fit covers every row, so a single row that can see both black lines
 * calibrates the rows that cannot. */
static float    s_wA;
static float    s_wB;
static float    s_widthModel[TRK_ROWS];
static uint32_t s_frames;

static void rebuild_width_model(void)
{
    uint8_t i;

    for (i = 0u; i < TRK_ROWS; i++)
    {
        float w = (s_wA * (float)s_rowY[i]) + s_wB;

        if (w < 6.0f)
        {
            w = 6.0f;
        }
        if (w > 400.0f)
        {
            w = 400.0f;
        }
        s_widthModel[i] = w;
    }
}

/* ---- small helpers ------------------------------------------------------ */

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

void Track_Init(void)
{
    float yNear = (float)s_rowY[0];
    float yFar  = (float)s_rowY[TRK_ROWS - 1u];
    float dy    = yNear - yFar;

    if (dy < 1.0f)
    {
        dy = 1.0f;
    }

    s_wA = (TRK_WIDTH_NEAR_PX - TRK_WIDTH_FAR_PX) / dy;
    s_wB = TRK_WIDTH_NEAR_PX - (s_wA * yNear);

    rebuild_width_model();
    s_frames = 0u;
}

float Track_LearnedWidth(uint8_t row)
{
    return (row < TRK_ROWS) ? s_widthModel[row] : 0.0f;
}

/*
 * Least squares slope of the corridor centre over a run of rows, expressed as
 * pixels of sideways movement per row of distance ahead. Positive = bends right.
 */
static bool fit_heading(const TrackModel *m, uint8_t from, uint8_t to, float *out)
{
    float   sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    uint8_t n = 0u;
    uint8_t i;
    float   den;

    for (i = from; (i <= to) && (i < TRK_ROWS); i++)
    {
        if (m->valid[i])
        {
            float d = m->y[0] - m->y[i]; /* distance ahead, grows with i */
            float c = m->center[i];

            sx += d;
            sy += c;
            sxx += d * d;
            sxy += d * c;
            n++;
        }
    }

    if (n < 2u)
    {
        return false;
    }

    den = ((float)n * sxx) - (sx * sx);
    if (fabsf(den) < 1e-3f)
    {
        return false;
    }

    *out = (((float)n * sxy) - (sx * sy)) / den;
    return true;
}

/* ---- main entry point --------------------------------------------------- */

bool Track_Update(const TrkSegment *segs, uint8_t n, TrackModel *out)
{
    Seg     v[TRK_MAX_SEGS];
    uint8_t nv = 0u;
    uint8_t i, j, pass;
    float   ref;
    float   alpha;
    bool    trusting; /* early frames: believe the camera over the width model */

    /* measurements collected this frame, for the width fit at the end */
    float   mY[TRK_ROWS];
    float   mW[TRK_ROWS];
    uint8_t mN = 0u;

    /* ---------------------------------------------------------------
     * 1. Clean up the raw segments.
     * Anything close to horizontal is a start line or an intersection bar, not a
     * track edge. Following one of those would send the car straight off the track,
     * so they are dropped before anything else looks at them.
     * -------------------------------------------------------------*/
    for (i = 0u; (i < n) && (nv < TRK_MAX_SEGS); i++)
    {
        float ax = segs[i].x0, ay = segs[i].y0;
        float bx = segs[i].x1, by = segs[i].y1;
        float dy, dx, len;

        if (ay < by) /* make sure the tail is the nearer end */
        {
            float t;
            t = ax; ax = bx; bx = t;
            t = ay; ay = by; by = t;
        }

        dy = ay - by; /* >= 0 */
        dx = bx - ax;

        if (dy < (float)TRK_MIN_VECTOR_DY)
        {
            continue;
        }

        len = sqrtf((dx * dx) + (dy * dy));
        if (len < TRK_MIN_VECTOR_LEN)
        {
            continue;
        }

        v[nv].xTail = ax;
        v[nv].yTail = ay;
        v[nv].xHead = bx;
        v[nv].yHead = by;
        v[nv].dxdy  = dx / (by - ay); /* by - ay is negative, so this is a true dx/dy */
        v[nv].len   = len;
        v[nv].span  = dy;
        nv++;
    }

    /* ---------------------------------------------------------------
     * 2. Walk the corridor outwards, one sample row at a time.
     * The reference column starts at the car and then follows the corridor centre
     * upward. That is what keeps the search on our own piece of track when another
     * part of the circuit is also in frame.
     * -------------------------------------------------------------*/
    trusting = (s_frames < (uint32_t)TRK_WIDTH_FAST_FRAMES);
    alpha    = trusting ? TRK_WIDTH_ALPHA_FAST : TRK_WIDTH_ALPHA;
    ref      = CAM_CENTER_X;

    for (i = 0u; i < TRK_ROWS; i++)
    {
        float y      = (float)s_rowY[i];
        float bestL  = 0.0f, bestR = 0.0f;
        float lenL   = 0.0f, lenR = 0.0f;
        bool  hasL   = false, hasR = false;
        float wModel = s_widthModel[i];

        out->y[i]     = y;
        out->sawL[i]  = false;
        out->sawR[i]  = false;
        out->valid[i] = false;

        /* Two passes. A segment that genuinely spans this row is worth far more than
         * one that has to be stretched to reach it, so the stretched ones are only
         * consulted when nothing real covers the row. */
        for (pass = 0u; pass < 2u; pass++)
        {
            for (j = 0u; j < nv; j++)
            {
                float x;
                float over  = 0.0f;
                float allow;

                if (y > v[j].yTail)
                {
                    over  = y - v[j].yTail;
                    allow = TRK_EXTRAP_NEAR_ROWS;
                }
                else if (y < v[j].yHead)
                {
                    over  = v[j].yHead - y;
                    allow = TRK_EXTRAP_FAR_ROWS;
                }
                else
                {
                    allow = 0.0f;
                }

                if (over > 0.0f)
                {
                    float bySpan = TRK_EXTRAP_SPAN_K * v[j].span;

                    if (pass == 0u)
                    {
                        continue; /* first pass only wants segments that really cover it */
                    }
                    if (allow > bySpan)
                    {
                        allow = bySpan;
                    }
                    if (over > allow)
                    {
                        continue;
                    }
                }

                x = v[j].xTail + ((y - v[j].yTail) * v[j].dxdy);

                /* Innermost candidate on each side of the reference wins: those two
                 * are the edges of the corridor the car is actually inside. */
                if (x < ref)
                {
                    if (!hasL || (x > bestL))
                    {
                        bestL = x;
                        lenL  = v[j].len;
                        hasL  = true;
                    }
                }
                else
                {
                    if (!hasR || (x < bestR))
                    {
                        bestR = x;
                        lenR  = v[j].len;
                        hasR  = true;
                    }
                }
            }

            if (hasL && hasR)
            {
                break; /* both edges found without stretching anything */
            }
        }

        if (hasL && hasR)
        {
            float w = bestR - bestL;
            bool  plausible =
                trusting || ((w >= (wModel * TRK_WIDTH_MIN_RATIO)) && (w <= (wModel * TRK_WIDTH_MAX_RATIO)));

            if (plausible)
            {
                out->xl[i]   = bestL;
                out->xr[i]   = bestR;
                out->sawL[i] = true;
                out->sawR[i] = true;
                /* Only a genuine two-sided measurement is allowed to teach the model. */
                mY[mN] = y;
                mW[mN] = w;
                mN++;
            }
            else
            {
                /* Width makes no sense - most likely one of the two is a line from
                 * another part of the circuit. Keep the longer, better supported
                 * edge and infer the other one. */
                if (lenL >= lenR)
                {
                    hasR = false;
                }
                else
                {
                    hasL = false;
                }
            }
        }

        if (!(out->sawL[i] && out->sawR[i]))
        {
            float wInf = s_widthModel[i] * TRK_ONE_EDGE_SHRINK;

            if (hasL && !hasR)
            {
                out->xl[i]   = bestL;
                out->xr[i]   = bestL + wInf;
                out->sawL[i] = true;
            }
            else if (hasR && !hasL)
            {
                out->xr[i]   = bestR;
                out->xl[i]   = bestR - wInf;
                out->sawR[i] = true;
            }
            else if (!hasL && !hasR)
            {
                continue; /* row stays invalid */
            }
            else
            {
                /* both flags survived but the pair was rejected above - cannot happen,
                 * guarded so the row is never left half filled */
                continue;
            }
        }

        out->width[i] = out->xr[i] - out->xl[i];

        /* Too narrow on screen means too far away to be worth believing. */
        if (out->width[i] < TRK_MIN_ROW_WIDTH_PX)
        {
            out->sawL[i] = false;
            out->sawR[i] = false;
            continue;
        }

        out->center[i] = 0.5f * (out->xl[i] + out->xr[i]);
        out->valid[i]  = true;
        ref            = out->center[i];
    }

    /* ---------------------------------------------------------------
     * 2b. Re-fit the width profile.
     * Two or more rows that saw both lines give a straight line fit. A single row
     * only moves the profile up or down and keeps the slope, which is still enough
     * to stay calibrated through a corner where only one row ever sees both edges.
     * -------------------------------------------------------------*/
    if (mN >= 2u)
    {
        float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f, den;

        for (i = 0u; i < mN; i++)
        {
            sx += mY[i];
            sy += mW[i];
            sxx += mY[i] * mY[i];
            sxy += mY[i] * mW[i];
        }

        den = ((float)mN * sxx) - (sx * sx);
        if (fabsf(den) > 1e-3f)
        {
            float a = (((float)mN * sxy) - (sx * sy)) / den;
            float b = (sy - (a * sx)) / (float)mN;

            /* The track has to look wider close up than far away. A fit that says
             * otherwise came from a bad frame, so keep the slope and only move the
             * offset. */
            if (a > 0.05f)
            {
                s_wA += alpha * (a - s_wA);
                s_wB += alpha * (b - s_wB);
            }
            else
            {
                float bOnly = mW[0] - (s_wA * mY[0]);
                s_wB += alpha * (bOnly - s_wB);
            }
            rebuild_width_model();
        }
    }
    else if (mN == 1u)
    {
        float bOnly = mW[0] - (s_wA * mY[0]);

        s_wB += alpha * (bOnly - s_wB);
        rebuild_width_model();
    }
    else
    {
        /* nothing measured this frame, keep the profile as it is */
    }

    s_frames++;

    /* ---------------------------------------------------------------
     * 3. Patch single row dropouts, then find how far the model is continuous.
     * The driver walks rows 0..topRow, so a gap has to be filled or cut off - it
     * must never step over a row it knows nothing about.
     * -------------------------------------------------------------*/
    for (i = 1u; (i + 1u) < TRK_ROWS; i++)
    {
        if (!out->valid[i] && out->valid[i - 1u] && out->valid[i + 1u])
        {
            out->xl[i]     = 0.5f * (out->xl[i - 1u] + out->xl[i + 1u]);
            out->xr[i]     = 0.5f * (out->xr[i - 1u] + out->xr[i + 1u]);
            out->width[i]  = out->xr[i] - out->xl[i];
            out->center[i] = 0.5f * (out->xl[i] + out->xr[i]);
            out->sawL[i]   = false;
            out->sawR[i]   = false;
            out->valid[i]  = true;
        }
    }

    /* The near rows are what the safety check leans on hardest, so if the camera came
     * up short down there, continue the two lines downward from the rows that did
     * work. Over the bottom of the frame that is only a handful of centimetres of
     * real track, which no black line bends much in. */
    if (!out->valid[0])
    {
        uint8_t lo = TRK_ROWS;

        for (i = 0u; i < TRK_ROWS; i++)
        {
            if (out->valid[i])
            {
                lo = i;
                break;
            }
        }

        if ((lo <= (uint8_t)TRK_FILL_DOWN_MAX) && ((lo + 1u) < TRK_ROWS) && out->valid[lo + 1u])
        {
            float sl, sr;
            /* Slope taken over as many of the lowest good rows as exist, so one noisy
             * row cannot tilt the whole extension. */
            uint8_t hi = (uint8_t)(lo + 2u);
            float   dy;

            if ((hi >= TRK_ROWS) || !out->valid[hi])
            {
                hi = (uint8_t)(lo + 1u);
            }

            dy = out->y[lo] - out->y[hi];
            if (fabsf(dy) < 0.5f)
            {
                dy = 1.0f;
            }
            sl = (out->xl[lo] - out->xl[hi]) / dy;
            sr = (out->xr[lo] - out->xr[hi]) / dy;

            for (i = 0u; i < lo; i++)
            {
                float down = out->y[i] - out->y[lo];

                out->xl[i]     = out->xl[lo] + (sl * down);
                out->xr[i]     = out->xr[lo] + (sr * down);
                out->width[i]  = out->xr[i] - out->xl[i];
                out->center[i] = 0.5f * (out->xl[i] + out->xr[i]);
                out->sawL[i]   = false;
                out->sawR[i]   = false;
                out->valid[i]  = true;
            }
        }
    }

    out->nValid    = 0u;
    out->topRow    = 0u;
    out->bothEdges = false;
    for (i = 0u; i < TRK_ROWS; i++)
    {
        if (!out->valid[i])
        {
            break;
        }

        /* The corridor may bend, but it may not jump. A step bigger than this means
         * the search latched onto a different piece of track, so everything from here
         * outward is thrown away rather than steered by. */
        if (i > 0u)
        {
            float step  = fabsf(out->center[i] - out->center[i - 1u]);
            float allow = TRK_MAX_CENTER_STEP_FRAC * out->width[i];

            if (step > allow)
            {
                break;
            }
        }

        out->nValid = (uint8_t)(i + 1u);
        out->topRow = i;
        if (out->sawL[i] && out->sawR[i])
        {
            out->bothEdges = true;
        }
    }

    /* Rows past the first gap are unusable even if the camera saw something there. */
    for (i = out->nValid; i < TRK_ROWS; i++)
    {
        out->valid[i] = false;
    }

    out->haveTrack = (out->nValid >= 2u);

    /* ---------------------------------------------------------------
     * 4. Safety margins and headings.
     * -------------------------------------------------------------*/
    for (i = 0u; i < TRK_ROWS; i++)
    {
        if (out->valid[i])
        {
            float m = SAFE_MARGIN_FRAC * out->width[i];

            if (m < SAFE_MARGIN_MIN_PX)
            {
                m = SAFE_MARGIN_MIN_PX;
            }
            /* A narrow corridor must still leave something to aim at: never let the
             * two margins meet and cross over each other. */
            if (m > (0.45f * out->width[i]))
            {
                m = 0.45f * out->width[i];
            }
            out->margin[i] = m;
        }
        else
        {
            out->margin[i] = SAFE_MARGIN_MIN_PX;
        }
    }

    out->headNear = 0.0f;
    out->headFar  = 0.0f;
    out->curv     = 0.0f;

    if (out->haveTrack)
    {
        float hn = 0.0f, hf = 0.0f;
        bool  okN, okF;

        okN = fit_heading(out, 0u, 3u, &hn);
        okF = fit_heading(out, 4u, (uint8_t)(TRK_ROWS - 1u), &hf);

        if (!okN)
        {
            /* Too few near rows: fall back to whatever the whole model says. */
            okN = fit_heading(out, 0u, (uint8_t)(TRK_ROWS - 1u), &hn);
            if (!okN)
            {
                hn = 0.0f;
            }
        }
        if (!okF)
        {
            /* Cannot see far enough to say anything new about the distance, so treat
             * the road ahead as a continuation of the road here. That keeps curv at
             * zero instead of inventing a corner that was never observed. */
            hf = hn;
        }

        out->headNear = clampf(hn, -4.0f, 4.0f);
        out->headFar  = clampf(hf, -4.0f, 4.0f);
        out->curv     = out->headFar - out->headNear;
    }

    return out->haveTrack;
}
