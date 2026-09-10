#include "intersection.h"
#include "race_config.h"
#include <math.h>
#include <stddef.h>

#define ISEC_MAX_EDGES 12u

/* One camera vector that runs up the track, described where it really is. */
typedef struct
{
    float f0, f1;  /* forward range it covers, cm. f0 is the end nearest the car */
    float lat0;    /* how far to the left of the car it sits at f0, cm           */
    float slope;   /* sideways per forward: 0 is parallel to the car             */
    float len;     /* ground length, cm                                          */
    bool  left;    /* which side of the car it is on                             */
} Edge;

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

/*
 * Image point -> the point on the track it came from, in centimetres.
 *
 * Flat ground and a pinhole camera, which between them make this two divisions.
 * How far below the vanishing point a row sits is inversely proportional to how
 * far ahead it is, and a column then scales into a lateral offset with that
 * distance. Positive lat is to the left, matching the image convention where
 * smaller columns are further left.
 *
 * Everything this module decides is a statement about the track - "the edges stop
 * for about one track width", "the far piece is parallel to the near one" - and
 * none of those survive being asked of the picture instead, where the same gap is
 * thirty rows deep at the bumper and three near the horizon.
 */
static bool to_ground(float u, float v, float *fwd, float *lat)
{
    float d = v - CAM_HORIZON_ROW; /* rows below the vanishing point */

    if (d < ISEC_MIN_ROWS_BELOW_HZ)
    {
        return false; /* at or near the horizon: no usable distance in it */
    }

    *fwd = (PIXY_FOCAL_PX * CAM_HEIGHT_CM) / d;
    *lat = (CAM_CENTER_X - u) * (*fwd) / PIXY_FOCAL_PX;
    return true;
}

static float lat_at(const Edge *e, float f)
{
    return e->lat0 + (e->slope * (f - e->f0));
}

/* ---- step 1: which vectors run up the track ----------------------------- */

/*
 * Sorts the raw frame into the vectors that are track edges and everything else.
 *
 * "Everything else" is mostly the bars across the mouth of a crossing, and this is
 * where they get thrown away - the whole method only ever looks at lines that run
 * away from the car, and where they stop.
 */
static uint8_t collect_edges(const TrkSegment *segs, uint8_t n, Edge *out,
                            float *barFwd, uint8_t *nBar)
{
    uint8_t i;
    uint8_t k = 0u;

    *nBar = 0u;

    const float vMin = CAM_HORIZON_ROW + ISEC_MIN_ROWS_BELOW_HZ;

    for (i = 0u; (i < n) && (k < ISEC_MAX_EDGES); i++)
    {
        float uN = segs[i].x0, vN = segs[i].y0;
        float uF = segs[i].x1, vF = segs[i].y1;
        float dF, dL, len2;
        float fNear, lNear, fFar, lFar;

        /* Order the ends so vN is the one lower in the picture, which is the one
         * nearer the car. */
        if (vF > vN)
        {
            float t;
            t = uN; uN = uF; uF = t;
            t = vN; vN = vF; vF = t;
        }

        if (vN < vMin)
        {
            continue; /* the whole thing sits in the unusable band by the horizon */
        }

        if (vF < vMin)
        {
            /*
             * The far end runs up past the last row worth unprojecting. Cut it
             * there rather than throwing the vector away: an edge that reaches to
             * the horizon is the most useful one in the frame, and dropping it
             * because of where it ends loses the near two thirds as well. This is
             * what makes the far side of a crossing visible at all - the camera
             * reports it as one long vector running off the top of the picture.
             */
            float t = (vN - vMin) / (vN - vF);

            uF = uN + ((uF - uN) * t);
            vF = vMin;
        }

        if (!to_ground(uN, vN, &fNear, &lNear) ||
            !to_ground(uF, vF, &fFar, &lFar))
        {
            continue;
        }

        dF = fFar - fNear;
        dL = lFar - lNear;

        /* Runs up the track, not across it. */
        if (dF < (ISEC_LONG_RATIO * fabsf(dL)))
        {
            /*
             * Lying across it instead. These are never steered by - that is the
             * whole point of sorting them out here - but where they are is kept,
             * because a line lying across the track ahead is the one piece of
             * positive evidence that the white space beyond the edges is a
             * crossing rather than the end of what the camera can see.
             */
            if ((fabsf(dL) > ISEC_MIN_BAR_CM) && (*nBar < ISEC_MAX_EDGES))
            {
                barFwd[*nBar] = 0.5f * (fNear + fFar);
                (*nBar)++;
            }
            continue;
        }

        len2 = (dF * dF) + (dL * dL);
        if (len2 < (ISEC_MIN_EDGE_CM * ISEC_MIN_EDGE_CM))
        {
            continue; /* too short on the ground to say where it goes */
        }

        /* Comes near the car's own piece of track. Only the near end is tested:
         * a straight edge seen from a car that is a few degrees off runs further
         * and further to one side the more of it you can see, and there is nothing
         * wrong with that. A line whose NEAREST point is already out at the edge
         * of the world belongs to another part of the circuit. */
        if (fabsf(lNear) > ISEC_SIDE_MAX_CM)
        {
            continue;
        }

        if (dF < 1.0e-3f)
        {
            continue;
        }

        out[k].f0    = fNear;
        out[k].f1    = fFar;
        out[k].lat0  = lNear;
        out[k].slope = dL / dF;
        out[k].len   = sqrtf(len2);
        out[k].left  = ((0.5f * (lNear + lFar)) > 0.0f);
        k++;
    }

    return k;
}

/* ---- step 2: look for the hole ------------------------------------------ */

/*
 * Marks off, bin by bin, how much of the track ahead one edge actually covers.
 *
 * Binned in centimetres rather than compared vector by vector, because the camera
 * reports one black line as a ragged chain of short vectors that overlap, abut, or
 * leave a pixel between them, and the bins make all three the same thing.
 */
static void build_cover(const Edge *e, uint8_t n, bool left, int8_t *owner)
{
    uint8_t b, i;

    for (b = 0u; b < ISEC_BINS; b++)
    {
        owner[b] = -1;
    }

    for (i = 0u; i < n; i++)
    {
        int lo, hi, j;

        if (e[i].left != left)
        {
            continue;
        }

        lo = (int)(e[i].f0 / ISEC_BIN_CM);
        hi = (int)(e[i].f1 / ISEC_BIN_CM);
        if (lo < 0)
        {
            lo = 0;
        }
        if (hi >= (int)ISEC_BINS)
        {
            hi = (int)ISEC_BINS - 1;
        }

        for (j = lo; j <= hi; j++)
        {
            /* Nearest owner wins, so the bin either side of the hole names the
             * piece the car is actually following. */
            if ((owner[j] < 0) || (e[i].f0 < e[owner[j]].f0))
            {
                owner[j] = (int8_t)i;
            }
        }
    }

    /* A single empty bin between two pieces of the same edge is the camera
     * breaking one line in two, not a crossing. Fill those in before scanning. */
    for (b = 1u; (b + 1u) < ISEC_BINS; b++)
    {
        if ((owner[b] < 0) && (owner[b - 1u] >= 0) && (owner[b + 1u] >= 0))
        {
            owner[b] = owner[b - 1u];
        }
    }
}

/*
 * Walks one side of the track outward from the car and asks whether the edge stops
 * and starts again.
 *
 * Returns true and fills gapStart/gapEnd when this side shows a crossing.
 */
static bool find_gap(const Edge *e, uint8_t n, bool left,
                     const int8_t *owner, const int8_t *other,
                     float *gapStart, float *gapEnd)
{
    uint8_t i;
    uint8_t b0, b1, b2;

    /* The near piece has to start near the car. An edge that only appears out in
     * the distance is not one the car is following. */
    b0 = 0u;
    while ((b0 < ISEC_BINS) && (owner[b0] < 0))
    {
        b0++;
    }
    if ((b0 >= ISEC_BINS) || (((float)b0 * ISEC_BIN_CM) > ISEC_EDGE_START_MAX_CM))
    {
        return false;
    }

    b1 = b0;
    while ((b1 < ISEC_BINS) && (owner[b1] >= 0))
    {
        b1++;
    }
    if (b1 >= ISEC_BINS)
    {
        return false; /* the edge runs off the end of the scan: no hole in it */
    }

    b2 = b1;
    while ((b2 < ISEC_BINS) && (owner[b2] < 0))
    {
        b2++;
    }

#if ISEC_REQUIRE_FAR_EDGE
    if (b2 >= ISEC_BINS)
    {
        /* The edge stops and nothing follows it. That is the camera running out of
         * look-ahead, or an edge leaving the side of the frame in a corner - both
         * of which happen constantly and neither of which is a crossing. */
        return false;
    }
#else
    if (b2 >= ISEC_BINS)
    {
        b2 = (uint8_t)(b1 + (uint8_t)(ISEC_GAP_MIN_CM / ISEC_BIN_CM));
    }
#endif

    /*
     * The other edge gets a veto. A crossing cuts both black lines at the same
     * place, so if the line on the far side of the car runs unbroken straight
     * through this hole, the hole is not a crossing - it is this edge dropping out
     * while the track carries on. That is the difference between a crossing and
     * the inside line disappearing off the side of the frame in a corner, and it
     * costs one pass over an array that has already been built.
     */
    for (i = b1; i < b2; i++)
    {
        if (other[i] < 0)
        {
            break;
        }
    }
    if (i >= b2)
    {
        return false; /* the opposite edge spans the gap: the track goes on */
    }

    *gapStart = (float)b1 * ISEC_BIN_CM;
    *gapEnd   = (float)b2 * ISEC_BIN_CM;

    {
        float gap = *gapEnd - *gapStart;

        if ((gap < ISEC_GAP_MIN_CM) || (gap > ISEC_GAP_MAX_CM))
        {
            return false; /* not the size of hole a crossing track leaves */
        }
    }

#if ISEC_REQUIRE_FAR_EDGE
    {
        const Edge *nr = NULL;
        const Edge *fr = NULL;

        /*
         * The longest piece on each side of the hole, not whichever one happens to
         * touch it. The camera breaks one black line into a chain, and the chunk
         * that ends at the mouth of a crossing can be two or three centimetres
         * long - long enough to say the edge is there, nowhere near long enough to
         * say which way it points. The longest piece is the one whose direction
         * can be trusted, and since both tests below extrapolate anyway, distance
         * from the hole costs nothing.
         */
        for (i = 0u; i < n; i++)
        {
            if (e[i].left != left)
            {
                continue;
            }
            if (e[i].f0 < *gapStart)
            {
                if ((nr == NULL) || (e[i].len > nr->len))
                {
                    nr = &e[i];
                }
            }
            else if (e[i].f0 >= (*gapEnd - ISEC_BIN_CM))
            {
                if ((fr == NULL) || (e[i].len > fr->len))
                {
                    fr = &e[i];
                }
            }
            else
            {
                /* inside the hole, which cannot happen after the scan above */
            }
        }

        if ((nr == NULL) || (fr == NULL))
        {
            return false;
        }

        /*
         * The far piece has to be a real length of black line, not a stub. This is
         * the guard that matters most when the camera calibration is out: with the
         * horizon or the height wrong every distance the scan measures is wrong
         * too, and what gets through is short noisy vectors that happen to land on
         * the far side of something. A crossing puts a whole track's worth of edge
         * over there.
         */
        if (fr->len < ISEC_MIN_FAR_CM)
        {
            return false;
        }

        /* Parallel, and in line. The same black line picked up again on the far
         * side satisfies both; a different line that happens to be over there
         * satisfies neither. */
        if (fabsf(fr->slope - nr->slope) > ISEC_PARALLEL_TOL)
        {
            return false;
        }
        if (fabsf(lat_at(fr, *gapEnd) - lat_at(nr, *gapEnd)) > ISEC_COLLINEAR_CM)
        {
            return false;
        }
    }
#endif

    return true;
}

/* ---- step 3: already at the mouth of one ------------------------------- */
#if ISEC_MOUTH_ENABLE

/*
 * The other thing a crossing looks like, once the car is nearly on top of one.
 *
 * From a distance the far side of the hole is in frame and step 2 finds it. Close
 * up it is not: the far edges are a few pixels tall at the very top of the picture
 * and the camera often does not report them at all. What is left is both black
 * lines stopping dead a short way ahead and nothing beyond either of them - which
 * is a crossing seen from its own doorstep, and it is also exactly what the driver
 * of this car sees at the moment it most needs to decide to go straight.
 *
 * BOTH sides have to do it, and at the same distance. That is what keeps a corner
 * out: in a corner the inside line leaves the side of the frame long before the
 * outside one runs out of look-ahead, so the two ends are nowhere near each other.
 * A crossing cuts both lines with the same straight edge, so they stop together.
 *
 * Returns true and fills endCm with how far ahead the two lines stop.
 */
static bool side_dead_end(const int8_t *owner, float *endCm)
{
    uint8_t b0, b1, b;

    b0 = 0u;
    while ((b0 < ISEC_BINS) && (owner[b0] < 0))
    {
        b0++;
    }
    if ((b0 >= ISEC_BINS) || (((float)b0 * ISEC_BIN_CM) > ISEC_EDGE_START_MAX_CM))
    {
        return false; /* nothing on this side, or nothing near the car */
    }

    b1 = b0;
    while ((b1 < ISEC_BINS) && (owner[b1] >= 0))
    {
        b1++;
    }
    if (b1 >= ISEC_BINS)
    {
        return false; /* runs to the end of the scan: this line is not stopping */
    }

    if (((float)b1 * ISEC_BIN_CM) > ISEC_MOUTH_CM)
    {
        return false; /* stops, but too far out to be a doorstep */
    }

    if (((float)(b1 - b0) * ISEC_BIN_CM) < ISEC_MOUTH_MIN_RUN_CM)
    {
        return false; /* a stub, not a line the car was following */
    }

    for (b = b1; b < ISEC_BINS; b++)
    {
        if (owner[b] >= 0)
        {
            return false; /* something out there, so step 2 owns this frame */
        }
    }

    *endCm = (float)b1 * ISEC_BIN_CM;
    return true;
}

#if ISEC_MOUTH_ONE_SIDED
static bool side_empty(const int8_t *owner)
{
    uint8_t b;

    for (b = 0u; b < ISEC_BINS; b++)
    {
        if (owner[b] >= 0)
        {
            return false;
        }
    }
    return true;
}

/* Steepest thing on one side, as a fraction: how far from parallel to the car the
 * line the camera can see actually runs. */
static float side_slope(const Edge *e, uint8_t n, bool left)
{
    const Edge *best = NULL;
    uint8_t     i;

    for (i = 0u; i < n; i++)
    {
        if (e[i].left != left)
        {
            continue;
        }
        if ((best == NULL) || (e[i].len > best->len))
        {
            best = &e[i];
        }
    }

    return (best != NULL) ? fabsf(best->slope) : 1.0e9f;
}
#endif /* ISEC_MOUTH_ONE_SIDED */

/*
 * Is there a black line lying across the track, at or beyond where the edges
 * stopped? That is the mouth of the crossing, or its far side, and it is the
 * difference between "the track is cut here" and "this is as far as I can see".
 */
static bool bar_across(const float *barFwd, uint8_t nBar, float mouthCm)
{
    uint8_t i;

    for (i = 0u; i < nBar; i++)
    {
        if ((barFwd[i] > (mouthCm - ISEC_BAR_SLACK_CM)) &&
            (barFwd[i] < (mouthCm + ISEC_BLIND_CROSS_CM + ISEC_BAR_SLACK_CM)))
        {
            return true;
        }
    }
    return false;
}

static bool find_mouth(const Edge *e, uint8_t nEdge,
                       const int8_t *coverL, const int8_t *coverR,
                       const float *barFwd, uint8_t nBar,
                       float *mouthCm, bool *left, bool *right)
{
    float lEnd = 0.0f, rEnd = 0.0f;
    bool  l    = side_dead_end(coverL, &lEnd);
    bool  r    = side_dead_end(coverR, &rEnd);

#if !ISEC_MOUTH_ONE_SIDED
    (void)e;
    (void)nEdge;
#endif

    if (l && r)
    {
        if (fabsf(lEnd - rEnd) > ISEC_MOUTH_SKEW_CM)
        {
            return false; /* one line gave up well before the other: not one cut */
        }

        *mouthCm = (lEnd < rEnd) ? lEnd : rEnd;

#if ISEC_MOUTH_NEEDS_BAR
        if (!bar_across(barFwd, nBar, *mouthCm))
        {
            return false;
        }
#endif
        *left  = true;
        *right = true;
        return true;
    }

#if ISEC_MOUTH_ONE_SIDED
    /*
     * One line stops and the other is not in the picture at all.
     *
     * This is the car arriving off centre: the far edge is outside a 60 degree
     * view until it is most of a metre away, so close to a crossing there is
     * genuinely only one line to see, and insisting on two would mean never
     * recognising a crossing the car is off line for - which is the case it most
     * needs to get right.
     *
     * With only one line there is no second opinion, so the line itself has to
     * look like a crossing approach: running very nearly parallel to the car. In a
     * hairpin the one line the camera can hold onto sweeps away across the frame,
     * and that is what ISEC_MOUTH_STRAIGHT refuses.
     */
    if (l && side_empty(coverR) &&
        (side_slope(e, nEdge, true) <= ISEC_MOUTH_STRAIGHT) &&
        bar_across(barFwd, nBar, lEnd))
    {
        *mouthCm = lEnd;
        *left    = true;
        *right   = false;
        return true;
    }
    if (r && side_empty(coverL) &&
        (side_slope(e, nEdge, false) <= ISEC_MOUTH_STRAIGHT) &&
        bar_across(barFwd, nBar, rEnd))
    {
        *mouthCm = rEnd;
        *left    = false;
        *right   = true;
        return true;
    }
#endif

    return false;
}

#endif /* ISEC_MOUTH_ENABLE */

/* ---- public interface --------------------------------------------------- */

void Intersection_Init(Intersection *st)
{
    st->seen       = false;
    st->sawLeft    = false;
    st->sawRight   = false;
    st->gapStartCm = 0.0f;
    st->gapEndCm   = 0.0f;
    st->gapCm      = 0.0f;
    st->slope      = 0.0f;
    st->nEdges     = 0u;
    st->atMouth    = false;
    st->steer      = 0.0f;
    st->authority  = 0.0f;
    st->phase      = ISEC_IDLE;
    st->crossing   = false;
    st->budgetM    = 0.0f;
    st->phaseMs    = 0.0f;
    st->hits       = 0u;
    st->count      = 0u;
}

void Intersection_Update(const TrkSegment *segs, uint8_t n, Intersection *st)
{
    Edge    edges[ISEC_MAX_EDGES];
    float   barFwd[ISEC_MAX_EDGES];
    uint8_t nEdge, nBar;
    float   lStart = 0.0f, lEnd = 0.0f;
    float   rStart = 0.0f, rEnd = 0.0f;

    st->seen     = false;
    st->sawLeft  = false;
    st->sawRight = false;

    nEdge      = collect_edges(segs, n, edges, barFwd, &nBar);
    st->nEdges = nEdge;

    /* ---------------------------------------------------------------
     * 1. Which way is the track pointing, as far as the car is concerned.
     * Length weighted, so a long confident edge outvotes a stub. This is the
     * number the steering nulls while crossing, and it is worth computing every
     * frame whether or not a crossing is in progress - inside one the near edges
     * have gone and this is coming off the far piece, which is the point.
     * -------------------------------------------------------------*/
    {
        float sum = 0.0f;
        float wgt = 0.0f;
        uint8_t i;

        for (i = 0u; i < nEdge; i++)
        {
            sum += edges[i].slope * edges[i].len;
            wgt += edges[i].len;
        }

        if (wgt > 1.0f)
        {
            st->slope = sum / wgt;
        }
    }

    /* ---------------------------------------------------------------
     * 2. Does either edge stop and start again?
     * -------------------------------------------------------------*/
    {
        int8_t coverL[ISEC_BINS];
        int8_t coverR[ISEC_BINS];

        build_cover(edges, nEdge, true, coverL);
        build_cover(edges, nEdge, false, coverR);

        st->sawLeft  = find_gap(edges, nEdge, true, coverL, coverR, &lStart, &lEnd);
        st->sawRight = find_gap(edges, nEdge, false, coverR, coverL, &rStart, &rEnd);

#if ISEC_MOUTH_ENABLE
        if (!st->sawLeft && !st->sawRight)
        {
            float mouth = 0.0f;
            bool  ml = false, mr = false;

            if (find_mouth(edges, nEdge, coverL, coverR, barFwd, nBar,
                           &mouth, &ml, &mr))
            {
                /* The line stops with nothing beyond. The far side cannot be
                 * measured from here, so it is assumed to be one track width
                 * away - which is what a crossing is. */
                st->sawLeft  = ml;
                st->sawRight = mr;
                lStart       = mouth;
                rStart       = mouth;
                lEnd         = mouth + ISEC_BLIND_CROSS_CM;
                rEnd         = lEnd;
                st->atMouth  = true;
            }
            else
            {
                st->atMouth = false;
            }
        }
        else
        {
            st->atMouth = false;
        }
#else
        st->atMouth = false;
#endif
    }

    if (st->sawLeft || st->sawRight)
    {
        st->seen = true;

        if (st->sawLeft && st->sawRight)
        {
            /* Both edges broken at once, which is what a crossing really is.
             * Take the nearer mouth and the further far side, so the latch
             * covers whichever side the car is closest to. */
            st->gapStartCm = (lStart < rStart) ? lStart : rStart;
            st->gapEndCm   = (lEnd > rEnd) ? lEnd : rEnd;
        }
        else if (st->sawLeft)
        {
            st->gapStartCm = lStart;
            st->gapEndCm   = lEnd;
        }
        else
        {
            st->gapStartCm = rStart;
            st->gapEndCm   = rEnd;
        }

        st->gapCm = st->gapEndCm - st->gapStartCm;
    }

    /* ---------------------------------------------------------------
     * 3. Line the car up with the track.
     * Parallel, not centred. Nulling the heading means the car leaves the
     * crossing on the line it entered on, which is what going straight over one
     * means; aiming at a centre would need the corridor width, and the corridor
     * width is exactly what cannot be measured inside a crossing.
     * -------------------------------------------------------------*/
    if (nEdge > 0u)
    {
        /* Positive slope means the track runs off to the left ahead, so the car is
         * pointing right of it and has to steer left. steer is positive right. */
        st->steer = clampf(-ISEC_HEAD_GAIN * st->slope, -ISEC_STEER_MAX, ISEC_STEER_MAX);
    }
    else
    {
        /* Nothing running up the track this frame - white space on all sides.
         * Fade toward straight ahead rather than holding the last command. */
        st->steer *= ISEC_ALIGN_DECAY;
    }

    /* ---------------------------------------------------------------
     * 4. Confidence, and the latch.
     * -------------------------------------------------------------*/
    if (st->phase == ISEC_COOLDOWN)
    {
        /* Just came out of one. The far side of the same crossing looks like the
         * near side of another, and latching onto that would take the car straight
         * through whatever corner comes next. */
        st->hits      = 0u;
        st->steer     = 0.0f;
        st->authority = 0.0f;
        st->crossing  = false;
        return;
    }

    if (st->seen)
    {
        if (st->hits < 200u)
        {
            st->hits++;
        }
    }
    else if (st->hits > 0u)
    {
        st->hits--;
    }
    else
    {
        /* nothing seen and nothing to forget */
    }

    if (st->phase == ISEC_CROSSING)
    {
        /* Committed. Authority stays whole until the distance budget runs out in
         * Intersection_Advance, whatever the camera does in the meantime - which
         * is the entire point, because a few frames from now the near edges will
         * have gone off the bottom of the picture and there will be nothing left
         * to make the decision from. */
        st->authority = 1.0f;
        st->crossing  = true;
        return;
    }

    st->crossing = false;

    if ((st->hits < (uint8_t)ISEC_CONFIRM_FRAMES) || !st->seen)
    {
        st->authority = 0.0f;
        return;
    }

    if (st->gapStartCm > ISEC_COMMIT_CM)
    {
        st->authority = 0.0f; /* seen, but still too far off to act on */
        return;
    }

    /*
     * Commit. How far to drive is not a guess: the mouth of the crossing is
     * gapStartCm ahead and the far side gapEndCm ahead, both measured, so the
     * latch runs to the far side plus the length of the car. The clamp is only
     * there so a silly measurement cannot buy a silly amount of blind driving.
     */
    st->phase     = ISEC_CROSSING;
    st->crossing  = true;
    st->authority = 1.0f;
    st->budgetM   = clampf((st->gapEndCm + ISEC_CLEAR_CM) * 0.01f,
                           ISEC_CROSS_MIN_M, ISEC_CROSS_MAX_M);
    st->phaseMs   = 0.0f;
    st->count++;
}

void Intersection_Advance(Intersection *st, float distM, float dt)
{
    if (distM < 0.0f)
    {
        distM = 0.0f; /* reversing does not un-cross an intersection */
    }
    if (dt < 0.0f)
    {
        dt = 0.0f;
    }

    if (st->phase == ISEC_IDLE)
    {
        st->phaseMs = 0.0f;
        return;
    }

    st->budgetM -= distM;
    st->phaseMs += dt * 1000.0f;

    /* Distance is what ends a crossing, because what limits how long the car may
     * drive on a latch is the ground it covers doing it, not a number of frames.
     * The timer is only the backstop for the one case distance cannot end: a car
     * that has stopped moving covers no ground and would sit latched forever. */
    if ((st->budgetM > 0.0f) && (st->phaseMs < ISEC_PHASE_MAX_MS))
    {
        return;
    }

    if (st->phase == ISEC_CROSSING)
    {
        st->phase   = ISEC_COOLDOWN;
        st->budgetM = ISEC_COOLDOWN_M;
    }
    else
    {
        st->phase   = ISEC_IDLE;
        st->budgetM = 0.0f;
    }

    st->phaseMs   = 0.0f;
    st->hits      = 0u;
    st->steer     = 0.0f;
    st->authority = 0.0f;
    st->crossing  = false;
}
