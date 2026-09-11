/*
 * features.c - see features.h for what this is and why it is shaped this way.
 *
 * Everything below is measured on the ground in centimetres, not in the picture,
 * wherever the answer is a statement about the track rather than about the image.
 * intersection.c already makes this argument and it is worth repeating: the same
 * crossing is thirty image rows deep at the bumper and three near the horizon, so a
 * threshold in pixels means a different thing at every distance. A right angle on
 * the tarmac is a right angle from anywhere.
 *
 * The unprojection is the flat-ground pinhole one, two divisions, and it is the
 * exact inverse of the projection test/track_sim.c renders frames with.
 */
#include "features.h"
#include "race_config.h"

#include <math.h>
#include <string.h>

/* Two endpoints closer than this on the ground are treated as the same point, so
 * the segments through them are "meeting". A crossing corner is the only place two
 * long lines meet at a real angle; a track edge chain meets itself at nearly zero. */
#define VERTEX_NEAR_CM      22.0f

/* A hole in an edge shorter than this is the camera chopping one line into
 * chunks, not a crossing. A crossing is about one track width of nothing. */
#define GAP_MIN_CM          12.0f

/* An endpoint nearer the horizon than this carries no usable distance - one row of
 * quantisation moves it a large fraction of how far away it is. Same threshold
 * intersection.c uses, and for the same reason. Segments that fail it are dropped
 * rather than unprojected into nonsense. */
#define MIN_ROWS_BELOW_HZ   ISEC_MIN_ROWS_BELOW_HZ
#define FAR_CM              400.0f

#define MAX_SEG             FEAT_MAX_SEG

/* ------------------------------------------------------------------ */
/* small helpers                                                       */
/* ------------------------------------------------------------------ */

static float clampf(float v, float lo, float hi)
{
    if (!(v == v)) return 0.0f;   /* NaN: the only value that fails this test */
    if (v < lo)    return lo;
    if (v > hi)    return hi;
    return v;
}

/*
 * Image pixel -> ground, in centimetres ahead of and to the left of the camera.
 *
 * The mounting comes from Track_HorizonRow() and Track_CamHeightCm(), which the
 * car works out from the corridor width it has actually measured, not from the
 * constants in race_config.h - those are only the seed the model starts from.
 * This is exactly what intersection.c's to_ground() does, and the two must agree
 * or the features would describe a different track from the one the geometry
 * module is reasoning about.
 *
 * Returns false for a point too near the vanishing line to carry a distance.
 */
static bool unproject(float u, float v, float *fwd, float *lat)
{
    float hz    = Track_HorizonRow();
    float h     = Track_CamHeightCm();
    float below = v - hz;

    if (below < MIN_ROWS_BELOW_HZ)
    {
        return false;
    }

    *fwd = (PIXY_FOCAL_PX * h) / below;
    if (*fwd > FAR_CM)
    {
        *fwd = FAR_CM;
    }
    *lat = (CAM_CENTER_X - u) * (*fwd) / PIXY_FOCAL_PX;
    return true;
}

/* ------------------------------------------------------------------ */
/* per segment scratch                                                 */
/* ------------------------------------------------------------------ */

typedef struct
{
    float nf, nl;   /* near end on the ground: forward, lateral (cm)   */
    float ff, fl;   /* far end                                          */
    float df, dl;   /* far minus near, the direction it runs            */
    float lenPx;    /* length in the picture                            */
    float lenCm;    /* length on the ground                             */
    bool  up;       /* runs up the track rather than across it          */
    bool  left;     /* on the left of the camera axis                   */
} Seg;

/*
 * Largest hole along one side, in centimetres of forward distance.
 *
 * The segments on a side are forward intervals. Sorted by their near end and swept,
 * the biggest stretch that no interval covers is the gap - which is exactly the
 * white space a crossing leaves in an edge. Chunking noise is rejected by
 * GAP_MIN_CM, and an edge that simply ends produces no gap at all because there is
 * nothing beyond it to close one.
 */
static float largest_gap(const Seg *s, const int *idx, int n)
{
    int   order[MAX_SEG];
    int   i, j;
    float reach, best = 0.0f;

    if (n < 2) return 0.0f;

    for (i = 0; i < n; i++) order[i] = idx[i];

    /* insertion sort by near end - n is at most twelve */
    for (i = 1; i < n; i++)
    {
        int k = order[i];
        j = i - 1;
        while (j >= 0 && s[order[j]].nf > s[k].nf)
        {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = k;
    }

    reach = s[order[0]].ff;
    for (i = 1; i < n; i++)
    {
        float start = s[order[i]].nf;
        if (start - reach > best)
        {
            best = start - reach;
        }
        if (s[order[i]].ff > reach)
        {
            reach = s[order[i]].ff;
        }
    }

    return (best >= GAP_MIN_CM) ? best : 0.0f;
}

/* ------------------------------------------------------------------ */
/* the build                                                           */
/* ------------------------------------------------------------------ */

void Features_Build(const TrkSegment *segs, uint8_t n, const TrackModel *tm,
                    float *out)
{
    Seg   s[MAX_SEG];
    int   leftIdx[MAX_SEG], rightIdx[MAX_SEG];
    int   nLeft = 0, nRight = 0;
    int   nUp = 0, nCross = 0;
    float crossLenCm = 0.0f;
    float meanLenPx = 0.0f, maxLenPx = 0.0f;
    float topReach = 0.0f;
    float farLatMin = 1e9f, farLatMax = -1e9f;
    float slopeMin = 1e9f, slopeMax = -1e9f;
    float vertexMax = 0.0f;
    int   vertexN = 0;
    float reachLeft = 0.0f, reachRight = 0.0f;
    float barBeyond = 0.0f;
    float widthMax = 0.0f, widthSum = 0.0f;
    int   widthCnt = 0, sawL = 0, sawR = 0;
    int   nRaw = (n > MAX_SEG) ? MAX_SEG : (int)n;
    int   nSeg = 0;
    int   i, j, r;

    /*
     * ---- unproject every segment once ----
     *
     * Segments with an end up by the vanishing line are dropped here rather than
     * carried through with a made-up distance. That is one row of quantisation
     * moving a point by half the length of the track, and a feature built on it
     * would be noise dressed up as evidence.
     */
    for (i = 0; i < nRaw; i++)
    {
        float dxPx = segs[i].x1 - segs[i].x0;
        float dyPx = segs[i].y0 - segs[i].y1;   /* y0 is the near end, so this is +ve */

        if (!unproject(segs[i].x0, segs[i].y0, &s[nSeg].nf, &s[nSeg].nl) ||
            !unproject(segs[i].x1, segs[i].y1, &s[nSeg].ff, &s[nSeg].fl))
        {
            continue;
        }

        s[nSeg].df = s[nSeg].ff - s[nSeg].nf;
        s[nSeg].dl = s[nSeg].fl - s[nSeg].nl;

        s[nSeg].lenPx = sqrtf((dxPx * dxPx) + (dyPx * dyPx));
        s[nSeg].lenCm = sqrtf((s[nSeg].df * s[nSeg].df) + (s[nSeg].dl * s[nSeg].dl));

        /* Runs up the track rather than across it, tested on the ground exactly
         * as intersection.c tests it - ISEC_LONG_RATIO of 1.0 is 45 degrees, and
         * a crossing bar is nearer 90. Doing this in the picture instead would
         * call the same line different things at different distances. */
        s[nSeg].up = s[nSeg].df >= (ISEC_LONG_RATIO * fabsf(s[nSeg].dl));

        /* Which edge this belongs to, decided at the end nearest the car. Using
         * the midpoint instead splits one edge in two whenever the car is at an
         * angle and the far piece has crossed the camera axis - which is exactly
         * the oblique crossing this is supposed to find the hole in. */
        s[nSeg].left = s[nSeg].nl >= 0.0f;

        nSeg++;
    }

    for (i = 0; i < nSeg; i++)
    {
        meanLenPx += s[i].lenPx;
        if (s[i].lenPx > maxLenPx) maxLenPx = s[i].lenPx;

        if (s[i].up)
        {
            nUp++;
            if (s[i].left) leftIdx[nLeft++] = (int)i;
            else           rightIdx[nRight++] = (int)i;

            if (s[i].ff > topReach) topReach = s[i].ff;
            if (s[i].fl < farLatMin) farLatMin = s[i].fl;
            if (s[i].fl > farLatMax) farLatMax = s[i].fl;

            if (s[i].df > 1.0f)
            {
                float slope = s[i].dl / s[i].df;
                if (slope < slopeMin) slopeMin = slope;
                if (slope > slopeMax) slopeMax = slope;
            }

            if (s[i].left)  { if (s[i].ff > reachLeft)  reachLeft  = s[i].ff; }
            else            { if (s[i].ff > reachRight) reachRight = s[i].ff; }
        }
        else
        {
            nCross++;
            crossLenCm += s[i].lenCm;
        }
    }

    if (nSeg > 0) meanLenPx /= (float)nSeg;
    if (farLatMin > farLatMax) { farLatMin = 0.0f; farLatMax = 0.0f; }
    if (slopeMin > slopeMax)   { slopeMin = 0.0f; slopeMax = 0.0f; }

    /*
     * ---- vertices ----
     *
     * Two segments whose endpoints land on the same spot on the ground, with a real
     * angle between them, is the signature of the oblique approach: the near edge
     * running away from the car and the crossing's own edge cutting across it, both
     * reported as long vectors sharing a corner. |sin| of the angle between them
     * peaks at a right angle and falls to nothing when they are collinear, which is
     * what a chopped-up straight edge looks like.
     */
    for (i = 0; i < nSeg; i++)
    {
        for (j = i + 1; j < nSeg; j++)
        {
            float ends[4][2];
            int   e;
            float best = 1e9f;

            ends[0][0] = s[i].nf - s[j].nf; ends[0][1] = s[i].nl - s[j].nl;
            ends[1][0] = s[i].nf - s[j].ff; ends[1][1] = s[i].nl - s[j].fl;
            ends[2][0] = s[i].ff - s[j].nf; ends[2][1] = s[i].fl - s[j].nl;
            ends[3][0] = s[i].ff - s[j].ff; ends[3][1] = s[i].fl - s[j].fl;

            for (e = 0; e < 4; e++)
            {
                float d = (ends[e][0] * ends[e][0]) + (ends[e][1] * ends[e][1]);
                if (d < best) best = d;
            }

            if (best <= (VERTEX_NEAR_CM * VERTEX_NEAR_CM))
            {
                float la = s[i].lenCm, lb = s[j].lenCm;

                if (la > 3.0f && lb > 3.0f)
                {
                    float cross = (s[i].df * s[j].dl) - (s[i].dl * s[j].df);
                    float sinA  = fabsf(cross) / (la * lb);

                    if (sinA > 1.0f) sinA = 1.0f;
                    if (sinA > vertexMax) vertexMax = sinA;
                    if (sinA > 0.45f) vertexN++;   /* past ~27 degrees, a real corner */
                }
            }
        }
    }

    /*
     * ---- is anything lying across the road at or beyond where the edges stop ----
     *
     * The bars at the mouth of a crossing are never steered by - intersection.c is
     * emphatic about that - but they are evidence, and this is the one place they
     * are used. A cross-track segment sitting at the distance the edges gave out is
     * a crossing; the same distance with nothing across it is an edge that ran out
     * of frame.
     */
    {
        float stopAt = (reachLeft < reachRight) ? reachLeft : reachRight;

        for (i = 0; i < nSeg; i++)
        {
            if (!s[i].up && s[i].lenCm > 6.0f)
            {
                float mid = (s[i].nf + s[i].ff) * 0.5f;
                if (mid >= (stopAt - 15.0f))
                {
                    barBeyond = 1.0f;
                    break;
                }
            }
        }
    }

    /*
     * ---- how much wider the corridor is than it should be ----
     *
     * Not max width over mean width: perspective alone makes row 0 the widest
     * row in every frame ever taken, so that ratio is a constant and says
     * nothing. The question is whether the corridor is wider THAN IT SHOULD BE
     * at that distance, and track.c already knows the answer - Track_LearnedWidth
     * is the profile the car has measured for itself over the whole run.
     *
     * A crossing pushes this well above one, because the edges have stopped and
     * the corridor is being fitted across the mouth of the junction.
     */
    for (r = 0; r < TRK_ROWS; r++)
    {
        if (tm->valid[r])
        {
            float expect = Track_LearnedWidth((uint8_t)r);

            if (expect > 1.0f)
            {
                float ratio = tm->width[r] / expect;

                widthSum += ratio;
                widthCnt++;
                if (ratio > widthMax) widthMax = ratio;
            }
        }
        if (tm->sawL[r]) sawL++;
        if (tm->sawR[r]) sawR++;
    }

    /* ---- write it out ---- */

    for (r = 0; r < TRK_ROWS; r++)
    {
        out[FEAT_CENTER0 + r] = tm->valid[r]
                              ? clampf((tm->center[r] - CAM_CENTER_X) / CAM_CENTER_X, -2.0f, 2.0f)
                              : 0.0f;
        out[FEAT_WIDTH0 + r]  = tm->valid[r]
                              ? clampf(tm->width[r] / 45.0f, 0.0f, 3.0f)
                              : 0.0f;
        out[FEAT_VALID0 + r]  = tm->valid[r] ? 1.0f : 0.0f;
    }

    out[FEAT_HEAD_NEAR]    = clampf(tm->headNear / 2.0f, -3.0f, 3.0f);
    out[FEAT_HEAD_FAR]     = clampf(tm->headFar  / 2.0f, -3.0f, 3.0f);
    out[FEAT_CURV]         = clampf(tm->curv     / 2.0f, -3.0f, 3.0f);
    out[FEAT_ABS_HEAD_FAR] = clampf(fabsf(tm->headFar) / 2.0f, 0.0f, 3.0f);
    out[FEAT_ABS_CURV]     = clampf(fabsf(tm->curv)    / 2.0f, 0.0f, 3.0f);
    out[FEAT_NVALID]       = (float)tm->nValid / (float)TRK_ROWS;

    out[FEAT_NSEG]         = (float)nSeg / (float)MAX_SEG;
    out[FEAT_MEAN_LEN]     = clampf(meanLenPx / 40.0f, 0.0f, 3.0f);
    out[FEAT_MAX_LEN]      = clampf(maxLenPx  / 40.0f, 0.0f, 3.0f);
    out[FEAT_N_CROSS]      = (float)nCross / (float)MAX_SEG;
    out[FEAT_CROSS_LEN]    = clampf(crossLenCm / 80.0f, 0.0f, 3.0f);
    out[FEAT_N_UP]         = (float)nUp / (float)MAX_SEG;
    out[FEAT_SLOPE_SPREAD] = clampf((slopeMax - slopeMin) / 2.0f, 0.0f, 3.0f);
    out[FEAT_TOP_REACH]    = clampf(topReach / 200.0f, 0.0f, 2.0f);
    out[FEAT_FAR_SPREAD]   = clampf((farLatMax - farLatMin) / 100.0f, 0.0f, 3.0f);

    out[FEAT_VERTEX_MAX]   = clampf(vertexMax, 0.0f, 1.0f);
    out[FEAT_VERTEX_N]     = clampf((float)vertexN / 4.0f, 0.0f, 3.0f);
    out[FEAT_GAP_LEFT]     = clampf(largest_gap(s, leftIdx,  nLeft)  / 100.0f, 0.0f, 3.0f);
    out[FEAT_GAP_RIGHT]    = clampf(largest_gap(s, rightIdx, nRight) / 100.0f, 0.0f, 3.0f);
    out[FEAT_STOP_SKEW]    = clampf(fabsf(reachLeft - reachRight) / 100.0f, 0.0f, 3.0f);
    out[FEAT_STOP_FWD]     = clampf(((reachLeft < reachRight) ? reachLeft : reachRight)
                                    / 200.0f, 0.0f, 2.0f);
    out[FEAT_BAR_BEYOND]   = barBeyond;
    out[FEAT_WIDTH_BLOWUP] = (widthCnt > 0) ? clampf(widthMax, 0.0f, 4.0f) : 0.0f;
    out[FEAT_SAW_ASYM]     = (float)((sawL > sawR) ? (sawL - sawR) : (sawR - sawL))
                           / (float)TRK_ROWS;
}

void Features_History(const float *feat, float *out)
{
    out[FHIST_HEAD_FAR]     = feat[FEAT_HEAD_FAR];
    out[FHIST_CURV]         = feat[FEAT_CURV];
    out[FHIST_VERTEX_MAX]   = feat[FEAT_VERTEX_MAX];
    out[FHIST_CROSS_LEN]    = feat[FEAT_CROSS_LEN];
    out[FHIST_GAP_LEFT]     = feat[FEAT_GAP_LEFT];
    out[FHIST_GAP_RIGHT]    = feat[FEAT_GAP_RIGHT];
    out[FHIST_WIDTH_BLOWUP] = feat[FEAT_WIDTH_BLOWUP];
    out[FHIST_NVALID]       = feat[FEAT_NVALID];
}

/* Plain table, no snprintf: this file is compiled into the firmware and dragging
 * stdio in for the sake of a debug string is not a trade worth making. */
const char *Features_Name(int idx)
{
    static const char *const names[FEAT_N] = {
        "center0", "center1", "center2", "center3",
        "center4", "center5", "center6", "center7",
        "width0",  "width1",  "width2",  "width3",
        "width4",  "width5",  "width6",  "width7",
        "valid0",  "valid1",  "valid2",  "valid3",
        "valid4",  "valid5",  "valid6",  "valid7",
        "headNear", "headFar", "curv", "absHeadFar", "absCurv", "nValid",
        "nSeg", "meanLen", "maxLen", "nCross", "crossLen", "nUp",
        "slopeSpread", "topReach", "farSpread",
        "vertexMax", "vertexN", "gapLeft", "gapRight",
        "stopSkew", "stopFwd", "barBeyond", "widthBlowup", "sawAsym"
    };

    return (idx >= 0 && idx < FEAT_N) ? names[idx] : "?";
}
