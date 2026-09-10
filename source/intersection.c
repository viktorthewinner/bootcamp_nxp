#include "intersection.h"
#include "race_config.h"
#include <math.h>

#define XSEC_MAX_BARS 16

typedef struct
{
    float y;  /* image row the bar sits on   */
    float xa; /* left end                    */
    float xb; /* right end                   */
} Bar;

static XsecState s;
static uint8_t   s_miss;

/* The camera's own verdict for this frame. Refreshed by main.c every frame; the
 * simulator never touches it, so it is false throughout every simulated run. */
static bool      s_camSeen;
static float     s_camX, s_camY;
static uint8_t   s_camBranch;
static bool      s_camUsed;

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/*
 * Corridor centre and width at any image row, from the eight sample rows the
 * track model carries. The bars usually sit above the farthest sample row -
 * which is exactly where a crossing is while there is still time to do
 * something about it - so this has to extrapolate past the top of the model as
 * well as interpolate inside it.
 */
static bool corridor_at(const TrackModel *m, float y, float *cx, float *w)
{
    uint8_t i, top;

    if (!m->haveTrack || (m->nValid < 2u))
    {
        return false;
    }
    top = (uint8_t)(m->nValid - 1u);

    /* y falls as the row index grows: row 0 is the nearest to the bumper. */
    for (i = 0u; i < top; i++)
    {
        float ya = m->y[i];
        float yb = m->y[i + 1u];

        if ((y <= ya) && (y >= yb))
        {
            float d = ya - yb;
            float t;

            if (d < 0.5f)
            {
                d = 0.5f;
            }
            t   = (ya - y) / d;
            *cx = m->center[i] + (t * (m->center[i + 1u] - m->center[i]));
            *w  = m->width[i] + (t * (m->width[i + 1u] - m->width[i]));
            if (*w < 4.0f)
            {
                *w = 4.0f;
            }
            return true;
        }
    }

    if (y < m->y[top])
    {
        /* Farther out than the model reaches: continue the last two rows. Width
         * shrinks with distance, so it is floored rather than allowed to pass
         * through zero and come back negative. */
        float ya = m->y[top - 1u];
        float yb = m->y[top];
        float d  = ya - yb;
        float t;

        if (d < 0.5f)
        {
            d = 0.5f;
        }
        t   = (ya - y) / d;
        *cx = m->center[top - 1u] + (t * (m->center[top] - m->center[top - 1u]));
        *w  = m->width[top - 1u] + (t * (m->width[top] - m->width[top - 1u]));
        if (*w < 4.0f)
        {
            *w = 4.0f;
        }
        return true;
    }

    /* Nearer than the bottom sample row. */
    *cx = m->center[0];
    *w  = (m->width[0] < 4.0f) ? 4.0f : m->width[0];
    return true;
}

/*
 * Are the two lines splaying apart with distance?
 *
 * This is the strongest thing in the module, and it is strong because it tests
 * something that cannot happen. Two parallel track edges seen in perspective
 * ALWAYS converge: whatever the camera height, whatever the tilt, whatever the
 * lens, the pair is narrower on screen the farther away it is. There is no
 * mounting and no track width that makes a straight pair get wider with
 * distance.
 *
 * So a vector whose far end has moved OUTWARD from where its near end sits is
 * not following our line any more. At a crossing that is exactly what happens:
 * our line runs into the crossing track's line, and the camera follows the paint
 * round the corner and back out again, so the vector bends away instead of
 * closing in. Both sides do it at once and their top ends spread apart.
 *
 * Each vector is judged on its own two endpoints, which is the whole trick. An
 * earlier version measured the total spread of everything in the frame at a near
 * row and a far row; that reads whatever else happens to be in shot - the far
 * side of a hairpin, most often - and called ordinary straights divergent by
 * half again. A vector's own two ends cannot be contaminated by another piece of
 * track, because they belong to one piece of paint.
 */
static bool diverging(const TrkSegment *segs, uint8_t n, const TrackModel *m,
                      float *splay)
{
    uint8_t i;
    float   bestL = 0.0f, bestR = 0.0f; /* longest vector found on each side */
    float   upL = 0.0f, upR = 0.0f;     /* how each one moves per row upward */
    bool    haveL = false, haveR = false;

    (void)m;
    *splay = 0.0f;

    for (i = 0u; i < n; i++)
    {
        float ax = segs[i].x0, ay = segs[i].y0;
        float bx = segs[i].x1, by = segs[i].y1;
        float tx, ty, hx, hy, dy, len, up;

        /* tail is the end nearest the car, head the end farthest away */
        if (ay >= by)
        {
            tx = ax; ty = ay; hx = bx; hy = by;
        }
        else
        {
            tx = bx; ty = by; hx = ax; hy = ay;
        }

        dy = ty - hy;
        if (dy < XSEC_DIV_MIN_DY)
        {
            continue; /* too flat to have a meaningful direction */
        }

        len = dy;
        up  = (hx - tx) / dy; /* columns moved per row of going farther away */

        if (tx < CAM_CENTER_X)
        {
            if (!haveL || (len > bestL))
            {
                bestL = len; upL = up; haveL = true;
            }
        }
        else
        {
            if (!haveR || (len > bestR))
            {
                bestR = len; upR = up; haveR = true;
            }
        }
    }

    if (!haveL || !haveR)
    {
        return false;
    }

    /*
     * Outward is negative on the left and positive on the right, so the splay
     * is how far both of them move the wrong way. Taking the smaller of the two
     * means one line wandering does not make a junction on its own - they have
     * to disagree with perspective together, which is the thing that cannot
     * happen by accident.
     */
    *splay = (-upL < upR) ? -upL : upR;

    return (*splay >= XSEC_DIV_MIN_SLOPE);
}

void Xsec_CameraHint(bool seen, float x, float y, uint8_t branches)
{
    s_camSeen   = seen;
    s_camX      = x;
    s_camY      = y;
    s_camBranch = branches;
}

/*
 * Does the camera's reported junction sit on the corner we found?
 *
 * Requiring the position to agree, rather than just the fact of a junction
 * somewhere in frame, is what keeps this a corroboration. The camera flagging
 * something at the far end of a lap does not get to vouch for a corner here.
 */
static bool cam_backs(float vx, float vy)
{
    float dx, dy;

    if (!s_camSeen || (s_camBranch < (uint8_t)XSEC_CAM_MIN_BRANCH))
    {
        return false;
    }
    dx = s_camX - vx;
    dy = s_camY - vy;
    if (((dx * dx) + (dy * dy)) > (XSEC_CAM_NEAR * XSEC_CAM_NEAR))
    {
        return false;
    }
    s_camUsed = true;
    return true;
}

/*
 * One corner: a line of ours ending where another line sets off sideways.
 */
typedef struct
{
    bool  have;
    float stemDy;   /* rows the stem runs; the longest is our own line     */
    float stemLean; /* columns the stem moves per row of going away        */
    float tx, ty;   /* the stem's near end                                 */
    float vx, vy;   /* the corner itself - the stem's far end              */
    float armDx;    /* how far the arm reaches sideways from it, signed    */
    float armDy;    /* ...and how many rows it drops or climbs doing so    */
    float cosA;     /* cosine of the angle at the corner                   */
} Elbow;

/* The longest line running away from the car on one side of the corridor,
 * whatever it ends in. It is our own edge, and the straight line through it
 * is what the rest of that edge is measured against. */
typedef struct
{
    bool  have;
    float dy;
    float lean;   /* columns per row of going away             */
    float hx, hy; /* far end                                   */
    uint8_t idx;
} Stem;

/* Tail nearest the car, head farthest away. */
static void ends_of(const TrkSegment *s, float *tx, float *ty, float *hx, float *hy)
{
    if (s->y0 >= s->y1)
    {
        *tx = s->x0; *ty = s->y0; *hx = s->x1; *hy = s->y1;
    }
    else
    {
        *tx = s->x1; *ty = s->y1; *hx = s->x0; *hy = s->y0;
    }
}

/*
 * How far, in columns, the rest of one edge strays from the straight line
 * through its stem, toward a given side.
 *
 * A straight edge that the camera happened to cut in pieces keeps every piece
 * on one line, so this comes back near zero however many pieces there are. A
 * bend does not: each piece beyond the stem leans further into the corner
 * than the stem does, so its far end drifts off the stem's line, and the
 * drift grows with every row - a flat piece running across the top of the
 * frame lands tens of columns away from where the stem was heading. Measured
 * in columns rather than as a change of slope so that a two-row piece cannot
 * be thrown out for being too short to have a slope, and so that the
 * quantisation of a short piece (half a column at each end) cannot masquerade
 * as a bend.
 *
 * toward is +1 for rightward drift, -1 for leftward. Only pieces whose tail
 * sits on that edge's side of the corridor are looked at; the stem itself is
 * skipped.
 */
static float edge_drift(const TrkSegment *segs, uint8_t n, const TrackModel *m,
                        const Stem *st, bool rightSide, float toward)
{
    float   worst = 0.0f;
    uint8_t i;

    if (!st->have)
    {
        return 0.0f;
    }

    for (i = 0u; i < n; i++)
    {
        float tx, ty, hx, hy, cx, cw, pred, d;

        if (i == st->idx)
        {
            continue;
        }
        ends_of(&segs[i], &tx, &ty, &hx, &hy);
        if (!corridor_at(m, ty, &cx, &cw))
        {
            cx = CAM_CENTER_X;
        }
        if ((tx >= cx) != rightSide)
        {
            continue; /* the other edge's side */
        }
        if (hy > st->hy + 1.0f)
        {
            continue; /* entirely below the stem's far end: not the part beyond it */
        }

        /* Where the stem's straight line says this row should be. */
        pred = st->hx + (st->lean * (st->hy - hy));
        d    = toward * (hx - pred);
        if (d > worst)
        {
            worst = d;
        }
    }
    return worst;
}

/*
 * How far the elbow's own corner sits off the straight line through the piece
 * below its stem, toward the arm's side, in columns.
 *
 * Our line runs dead straight into a crossing, so a corner at the top of a
 * chain of pieces lies on the line the lower pieces set. The inside edge of a
 * bend curls: each piece leans further in than the one below, and the corner
 * at the top of it is the bend finally turning across the frame. Zero when the
 * stem is the nearest piece there is, which is what a real camera usually
 * gives at a crossing.
 */
static float pre_bend(const TrkSegment *segs, uint8_t n, const Elbow *e, float toward)
{
    float   worst = 0.0f;
    uint8_t i;

    for (i = 0u; i < n; i++)
    {
        float tx, ty, hx, hy, dy, jx, jy, pred, d;

        ends_of(&segs[i], &tx, &ty, &hx, &hy);
        dy = ty - hy;
        if (dy < XSEC_ELB_FAR_DY)
        {
            continue; /* its slope is extrapolated the length of the stem */
        }
        jx = hx - e->tx;
        jy = hy - e->ty;
        if (((jx * jx) + (jy * jy)) > (XSEC_ELB_JOIN * XSEC_ELB_JOIN))
        {
            continue; /* not the piece below the stem */
        }
        pred = hx + (((hx - tx) / dy) * (hy - e->vy));
        d    = toward * (e->vx - pred);
        if (d > worst)
        {
            worst = d;
        }
    }
    return worst;
}

/*
 * Follow one edge upward from a stem's far end through pieces that start where
 * the previous one stopped, and report where the chain finally ends.
 *
 * Only pieces that actually touch count, within XSEC_ELB_CHAIN. That is the
 * difference between our line resuming beyond a crossing - a separate piece
 * with the crossing track's width of bare floor between it and the stem - and
 * a line that simply carries on. A bend's outer edge carries on.
 */
static void chain_top(const TrkSegment *segs, uint8_t n, float hx, float hy,
                      float *topX, float *topY)
{
    uint8_t step, i;

    for (step = 0u; step < n; step++)
    {
        bool moved = false;

        for (i = 0u; i < n; i++)
        {
            float tx, ty, px, py, jx, jy;

            ends_of(&segs[i], &tx, &ty, &px, &py);
            if (py >= hy)
            {
                continue; /* does not go farther than where we are */
            }
            jx = tx - hx;
            jy = ty - hy;
            if (((jx * jx) + (jy * jy)) > (XSEC_ELB_CHAIN * XSEC_ELB_CHAIN))
            {
                continue; /* does not start where the last piece stopped */
            }
            hx    = px;
            hy    = py;
            moved = true;
        }
        if (!moved)
        {
            break;
        }
    }
    *topX = hx;
    *topY = hy;
}

/*
 * Does anything carry on through the corner in the direction the stem was
 * going? Our line STOPS at a crossing - that is what the corner is. A piece
 * leaving the corner along the stem's own direction means the line continues
 * and the sideways piece is something else joining it.
 */
static bool continues_on(const TrkSegment *segs, uint8_t n, const Elbow *e)
{
    float   sdx = e->vx - e->tx;
    float   sdy = e->ty - e->vy; /* rows going away, positive */
    float   slen = sqrtf((sdx * sdx) + (sdy * sdy));
    uint8_t i;

    if (slen < 1.0f)
    {
        return false;
    }
    for (i = 0u; i < n; i++)
    {
        float tx, ty, hx, hy, jx, jy, ax, ay, alen, c;

        ends_of(&segs[i], &tx, &ty, &hx, &hy);
        jx = tx - e->vx;
        jy = ty - e->vy;
        if (((jx * jx) + (jy * jy)) > (XSEC_ELB_CHAIN * XSEC_ELB_CHAIN))
        {
            continue; /* does not start at the corner */
        }
        ax   = hx - e->vx;
        ay   = e->vy - hy;
        alen = sqrtf((ax * ax) + (ay * ay));
        if (alen < 1.0f)
        {
            continue;
        }
        c = ((sdx * ax) + (sdy * ay)) / (slen * alen);
        if (c > XSEC_ELB_COS_TOL)
        {
            return true; /* heading on the way the stem was heading */
        }
    }
    return false;
}

/*
 * Does the other edge end where the crossing cuts it?
 *
 * Both our lines are cut by the same painted edge, so the line through the
 * corner along the arm passes through the far end of the other line too -
 * the other line stops there, and if it resumes it resumes beyond a gap. An
 * edge that runs on past that line without a break is not cut by anything:
 * it is the outer edge of a bend whose inner edge has turned across the frame
 * while the outer one, on its larger radius, is still on its way up. Rows
 * above the line, positive; the tolerance covers lens distortion bowing the
 * arm and the camera stopping a vector short of the paint.
 */
static float overrun(const TrkSegment *segs, uint8_t n, const Elbow *e, const Stem *other)
{
    float topX, topY, lineY;

    if (!other->have || (absf(e->armDx) < 1.0f))
    {
        return 0.0f;
    }
    chain_top(segs, n, other->hx, other->hy, &topX, &topY);
    lineY = e->vy + ((e->armDy / e->armDx) * (topX - e->vx));

    /*
     * A chain that runs out of the top of the frame has not been seen to
     * stop at all. Measured from where the frame cuts it, its overrun is
     * whatever it happens to be at row zero, which for a corner sitting in
     * the top few rows is never more than those few rows - so a bend whose
     * inner edge turns right at the top of the frame would pass on a
     * technicality. If the other line leaves the frame above the crossing
     * line, it was not cut by that line.
     */
    if ((topY <= XSEC_ELB_TOPCLIP) && ((lineY - topY) > 1.0f))
    {
        return XSEC_ELB_OVERRUN_ROWS + 1.0f;
    }
    return lineY - topY;
}

/*
 * Do our two lines each END in a right-angle corner, with the two corners
 * facing away from each other?
 *
 * This is what a crossing looks like when the tracker breaks the paint at the
 * corner instead of following it round: our line runs up the frame, stops, and
 * the crossing track's edge starts at the exact point it stopped and heads off
 * sideways. Two vectors sharing an endpoint. One on each side.
 *
 * The angle is the cue that names it, but the angle is not what makes it safe
 * to act on. Perspective does not preserve angles, so a square crossing can
 * measure anywhere from seventy to a hundred and twenty degrees depending on
 * where in the frame it lands and how the camera is tilted; a test built on
 * squareness alone would be a test built on the mounting.
 *
 * What makes it safe is the pair. A corner in our own track bends ONE way -
 * both its edges are two sides of the same road, so both arms sweep left, or
 * both sweep right. Nothing that is one road puts an arm out to the left and
 * another out to the right at the same moment. That only happens when a second
 * piece of track cuts across ours and our corridor punches a hole through the
 * middle of its edge, which is a crossing and nothing else.
 *
 * Each side is judged from its own two vectors, for the same reason the splay
 * test is: a corner belongs to one joint of one piece of paint and cannot be
 * contaminated by whatever else happens to be in shot.
 */
static bool elbowed(const TrkSegment *segs, uint8_t n, const TrackModel *m,
                    float *square)
{
    uint8_t i, j, k;
    Elbow   left, right;
    Stem    stemL, stemR;    /* our own line on each side, if described  */
    bool    rSweepL = false; /* right-hand edge bending sharply to the LEFT  */
    bool    lSweepR = false; /* left-hand edge bending sharply to the RIGHT  */

    left.have  = false;
    right.have = false;
    stemL.have = false;
    stemR.have = false;
    stemL.dy   = 0.0f;
    stemR.dy   = 0.0f;
    *square    = 0.0f;

    for (i = 0u; i < n; i++)
    {
        float tx, ty, hx, hy;  /* tail is nearest the car, head farthest */
        float sdx, sdy, slen, lean;
        float cx, cw;
        Stem *side;

        ends_of(&segs[i], &tx, &ty, &hx, &hy);

        sdy = ty - hy;
        if (sdy < XSEC_ELB_STEM_DY)
        {
            continue; /* not a line running away from us - not a stem */
        }
        sdx  = hx - tx;
        slen = sqrtf((sdx * sdx) + (sdy * sdy));
        if (slen < 1.0f)
        {
            continue;
        }
        lean = sdx / sdy;

        /*
         * Which side of the road the corner is on decides which elbow it can
         * be, and the corridor decides which side is which - not the middle of
         * the frame. An off-centre car or an off-axis camera can put both of
         * our lines the same side of the image, and a corner that could be
         * either elbow could pair with itself.
         */
        if (!corridor_at(m, hy, &cx, &cw))
        {
            cx = CAM_CENTER_X;
        }

        /*
         * Note that this edge exists at all, whether or not it turns into an
         * elbow. A lone elbow is only worth anything when the far side of the
         * road is described and says nothing against it; with nothing over
         * there, silence is not agreement.
         */
        side = (tx < cx) ? &stemL : &stemR;
        if (!side->have || (sdy > side->dy))
        {
            side->have = true;
            side->dy   = sdy;
            side->lean = lean;
            side->hx   = hx;
            side->hy   = hy;
            side->idx  = i;
        }

        for (j = 0u; j < n; j++)
        {
            float ex[2], ey[2];

            if (j == i)
            {
                continue;
            }
            ex[0] = segs[j].x0; ey[0] = segs[j].y0;
            ex[1] = segs[j].x1; ey[1] = segs[j].y1;

            /* Either end of the other vector may be the one at the corner. */
            for (k = 0u; k < 2u; k++)
            {
                float jx   = ex[k] - hx;
                float jy   = ey[k] - hy;
                float ax, ay, alen, cosA;
                Elbow *slot;

                if (((jx * jx) + (jy * jy)) > (XSEC_ELB_JOIN * XSEC_ELB_JOIN))
                {
                    continue; /* the two ends are not the same corner */
                }

                /* the arm runs from the corner out to that vector's free end */
                ax = ex[1u - k] - hx;
                ay = ey[1u - k] - hy;
                if (absf(ax) < XSEC_ELB_ARM_DX)
                {
                    continue; /* goes nowhere sideways: not a crossing edge */
                }
                alen = sqrtf((ax * ax) + (ay * ay));
                if (alen < 1.0f)
                {
                    continue;
                }

                /* stem points tail -> head, which is (sdx, -sdy) */
                cosA = (((sdx * ax) - (sdy * ay))) / (slen * alen);
                if (absf(cosA) > XSEC_ELB_COS_TOL)
                {
                    continue; /* not square enough to be a corner */
                }

                if ((ax < 0.0f) && (hx < cx))
                {
                    slot = &left;
                }
                else if ((ax > 0.0f) && (hx > cx))
                {
                    slot = &right;
                }
                else
                {
                    /*
                     * An arm pointing across the road rather than out of
                     * it. Not a crossing edge - but not nothing either.
                     * This is one side of our own track bending, and
                     * which way it bends is exactly what tells a lone
                     * elbow on the far side from a real junction.
                     */
                    if (ax < 0.0f)
                    {
                        rSweepL = true; /* right edge heading left  */
                    }
                    else
                    {
                        lSweepR = true; /* left edge heading right  */
                    }
                    continue;
                }

                /*
                 * A crossing edge lies across our path, so on screen it is
                 * close to level whatever the mounting: a line perpendicular
                 * to the direction of travel stays parallel to the horizon
                 * under any pitch, and a yaw of the car only tilts it a few
                 * degrees. An arm that climbs the frame steeply is a chord of
                 * our own edge going round a bend, not a crossing edge.
                 */
                if (absf(ay) > (XSEC_ELB_ARM_FLAT * absf(ax)))
                {
                    continue;
                }

                if (!slot->have || (sdy > slot->stemDy))
                {
                    slot->have     = true;
                    slot->stemDy   = sdy;
                    slot->stemLean = lean;
                    slot->tx       = tx;
                    slot->ty       = ty;
                    slot->vx       = hx;
                    slot->vy       = hy;
                    slot->armDx    = ax;
                    slot->armDy    = ay;
                    slot->cosA     = cosA;
                }
            }
        }
    }

    /*
     * A matched pair: the strongest reading, and the only one that rests on
     * geometry a single road cannot produce.
     */
    if (left.have && right.have)
    {
        /* Two corners at the same distance, far enough apart to be the two ends
         * of one edge with our track's width punched out of it. */
        if ((right.vx - left.vx) < XSEC_ELB_MIN_SEP)
        {
            return false;
        }
        if (absf(left.vy - right.vy) > XSEC_ELB_ROW_TOL)
        {
            return false;
        }

        /* 1 is a perfect right angle, 0 is a straight line. Reported for the
         * flight recorder; the arms pointing apart are what the verdict rests
         * on. */
        *square = 1.0f - ((absf(left.cosA) > absf(right.cosA)) ? absf(left.cosA)
                                                               : absf(right.cosA));
        return true;
    }

    if (!left.have && !right.have)
    {
        return false;
    }

    /*
     * One corner, and nothing opposite it.
     *
     * The camera does not always give both. Our line runs into the crossing and
     * turns, and the far side of the junction is out of frame, or too faint, or
     * simply not cut into vectors this time - so the frame carries one elbow and
     * a plain line on the other side. On the real track that is the COMMON
     * case, and it used to read as nothing at all, which is the expensive
     * answer: the car lifts for a corridor that has stopped, then the failsafe
     * stops it in the junction - or it steers off after the crossing's edge.
     *
     * What settles it is the rest of the road. A crossing is two straight lines
     * meeting; a bend is a road turning gradually, and a road turns as a whole.
     * So a lone corner is believed only when nothing else in the frame is
     * turning the way its arm points:
     *
     *   - the other edge must be there, and must not be bending toward the
     *     arm's side: neither sharply at one joint (rSweepL / lSweepR) nor
     *     gradually, piece by piece, which edge_drift measures. The inside
     *     edge of a bend seen from a distance turns across the top of the
     *     frame exactly like a crossing edge, and the only thing that gives it
     *     away is the outside edge turning with it.
     *   - the elbow's own line must have run straight into the corner, not
     *     curled into it piece by piece (pre_bend), and must stop there
     *     (continues_on).
     *   - the other edge must END on the line the arm draws across the road,
     *     because the same painted edge cuts both our lines (overrun). A
     *     tight bend seen from a tall mounting turns its inner edge inside
     *     the frame while its outer edge, on a larger radius, is still going
     *     straight up - and that outer edge runs on past the corner's row
     *     without a break, which no line cut by a crossing can do.
     *
     * Neither depends on the angle at the corner, which perspective makes
     * unreliable. Both are measured in columns against the straight line
     * through the edge's own nearest piece, so they hold at any mounting.
     */
    /*
     * And the corner must sit well inside the frame, not along its top edge.
     * A corner in the top rows is our line leaving the frame with something
     * level lying along the edge, and the crossing line the other edge would
     * have to end on leaves the frame with it - nothing above can be checked.
     * The inside edge of a tight bend seen from a tall mounting looks exactly
     * like that while its outside edge is still on its way up. The same
     * junction is back a few frames later, lower down, with everything in
     * view; waiting for that costs nothing.
     */
    if (left.have && stemR.have && !rSweepL && m->bothEdges &&
        (left.vy > XSEC_ELB_MIN_ROW) &&
        (edge_drift(segs, n, m, &stemR, true, -1.0f) < XSEC_ELB_SWEEP_COLS) &&
        (pre_bend(segs, n, &left, -1.0f) < XSEC_ELB_SWEEP_COLS) &&
        (overrun(segs, n, &left, &stemR) < XSEC_ELB_OVERRUN_ROWS) &&
        !continues_on(segs, n, &left))
    {
        /* The camera's own verdict is noted for the flight recorder either
         * way, and is the deciding vote only when XSEC_ELB_SINGLE is off. */
        bool cam = cam_backs(left.vx, left.vy);

        if (XSEC_ELB_SINGLE || cam)
        {
            *square = 1.0f - absf(left.cosA);
            return true;
        }
    }
    if (right.have && stemL.have && !lSweepR && m->bothEdges &&
        (right.vy > XSEC_ELB_MIN_ROW) &&
        (edge_drift(segs, n, m, &stemL, false, 1.0f) < XSEC_ELB_SWEEP_COLS) &&
        (pre_bend(segs, n, &right, 1.0f) < XSEC_ELB_SWEEP_COLS) &&
        (overrun(segs, n, &right, &stemL) < XSEC_ELB_OVERRUN_ROWS) &&
        !continues_on(segs, n, &right))
    {
        bool cam = cam_backs(right.vx, right.vy);

        if (XSEC_ELB_SINGLE || cam)
        {
            *square = 1.0f - absf(right.cosA);
            return true;
        }
    }

    return false;
}

void Xsec_Init(void)
{
    s.phase      = XSEC_IDLE;
    s.recognised = false;
    s.bothSides  = false;
    s.spanning   = false;
    s.bars       = 0u;
    s.barY       = 0.0f;
    s.cover      = 0.0f;
    s.gap        = 0.0f;
    s.diverge    = 0.0f;
    s.square     = 0.0f;
    s.camAgreed  = false;
    s.holdSteer  = 0.0f;
    s.holdYaw    = 0.0f;
    s.steerOut   = 0.0f;
    s.runM       = 0.0f;
    s.seenM      = 99.0f;
    s.agree      = 0u;
    s.count      = 0u;
    s_miss       = 0u;
}

bool Xsec_Holding(const XsecState *st)
{
    return (st->phase == XSEC_CROSSING);
}

bool Xsec_KeepPower(const XsecState *st)
{
    return (st->phase == XSEC_AHEAD) || (st->phase == XSEC_CROSSING);
}

uint8_t Xsec_ForTrack(const XsecState *st, const TrkSegment *in, uint8_t n,
                      TrkSegment *out)
{
    uint8_t i, k = 0u;

    if (n > (uint8_t)XSEC_MAX_SEGS)
    {
        n = (uint8_t)XSEC_MAX_SEGS;
    }
    for (i = 0u; i < n; i++)
    {
#if XSEC_ENABLE
        if (st->phase != XSEC_IDLE)
        {
            float dx = absf(in[i].x1 - in[i].x0);
            float dy = absf(in[i].y1 - in[i].y0);

            if (dx > (XSEC_TRACK_FLAT * dy))
            {
                continue; /* lies across the road: the crossing's edge, not ours */
            }
        }
#else
        (void)st;
#endif
        out[k] = in[i];
        k++;
    }
    return k;
}

/*
 * Freeze the steering for the drive through.
 *
 * What is frozen is the filtered approach angle, scaled by XSEC_HOLD_STEER_K.
 * Alongside it goes a reading of how much of that angle was correcting a yaw
 * rather than following the road: the near heading says which way the road
 * under the bumper points relative to the car, and a car steering hard while
 * the road under it is dead straight ahead is following a bend, whereas one
 * steering hard while the road under it points sideways is straightening up.
 * The two need opposite treatment inside the junction and this is the only
 * cue that separates them - see steerOut below.
 */
static void latch(float steerNow, float headNow)
{
    float y = absf(headNow) / XSEC_HOLD_YAW_FULL;

    if (y > 1.0f)
    {
        y = 1.0f;
    }
    s.holdSteer = steerNow * XSEC_HOLD_STEER_K;
    s.holdYaw   = y;
    s.steerOut  = s.holdSteer;
    s.runM      = 0.0f;
    s.count++;
}

void Xsec_Update(const TrkSegment *segs, uint8_t n, const TrackModel *m,
                 float travelM, float steerNow, float headNow, XsecState *out)
{
#if XSEC_ENABLE
    Bar     bar[XSEC_MAX_BARS];
    uint8_t nb = 0u;
    uint8_t i, j;

    float   bestCover = 0.0f;
    float   bestY     = 0.0f;
    float   bestW     = 0.0f;
    float   bestGap   = 0.0f;
    uint8_t bestN     = 0u;
    bool    bestL = false, bestR = false, bestSpan = false;
    bool    recog = false;

    /* ---------------------------------------------------------------
     * 1. Pull the sideways vectors back out of the pile.
     * These are the ones track.c throws away. A crossing edge is flat and long;
     * a track edge running away from the camera is neither.
     * -------------------------------------------------------------*/
    for (i = 0u; (i < n) && (nb < XSEC_MAX_BARS); i++)
    {
        float ax = segs[i].x0, ay = segs[i].y0;
        float bx = segs[i].x1, by = segs[i].y1;
        float dy = absf(ay - by);
        float dx = absf(bx - ax);

        if (dy > XSEC_BAR_MAX_DY)
        {
            continue;
        }
        if (dx < XSEC_BAR_MIN_DX)
        {
            continue;
        }

        bar[nb].y  = 0.5f * (ay + by);
        bar[nb].xa = (ax < bx) ? ax : bx;
        bar[nb].xb = (ax < bx) ? bx : ax;
        nb++;
    }

    /* ---------------------------------------------------------------
     * 2. Bars at the same distance ahead belong to the same crossing.
     * Take the group covering the most ground and describe it: how wide it is
     * in corridor widths, and whether it reaches out on both sides of us or
     * straight across the middle.
     * -------------------------------------------------------------*/
    for (i = 0u; i < nb; i++)
    {
        float   gy    = bar[i].y;
        float   cover = 0.0f, ccx, cw;
        uint8_t cnt   = 0u;
        bool    hasL = false, hasR = false, span = false;
        /* Innermost ends of the material either side of us - the two edges of
         * the hole our own track cuts in the crossing's lines. */
        float   innerL = -1.0e9f, innerR = 1.0e9f;

        if (!corridor_at(m, gy, &ccx, &cw))
        {
            continue;
        }

        for (j = 0u; j < nb; j++)
        {
            if (absf(bar[j].y - gy) > XSEC_BAR_ROW_TOL)
            {
                continue;
            }

            cover += bar[j].xb - bar[j].xa;
            cnt++;

            if (bar[j].xa < (ccx - (XSEC_SIDE_FRAC * cw)))
            {
                hasL = true;
            }
            if (bar[j].xb > (ccx + (XSEC_SIDE_FRAC * cw)))
            {
                hasR = true;
            }
            if ((bar[j].xa < ccx) && (bar[j].xb > ccx))
            {
                span = true;
            }

            if ((bar[j].xb < ccx) && (bar[j].xb > innerL))
            {
                innerL = bar[j].xb;
            }
            if ((bar[j].xa > ccx) && (bar[j].xa < innerR))
            {
                innerR = bar[j].xa;
            }
        }

        /*
         * The gap test, and the one that finally tells a junction from a corner.
         *
         * A crossing is two tracks painted on the same floor, so ours cuts a
         * hole about one track wide in the other one and leaves its edges as
         * stubs either side. That hole is the signature. Two track edges
         * flattening out as they converge on the vanishing point - which is
         * what the top row of any corner looks like - reach toward each other
         * instead of away, and leave no hole at all.
         */
        if ((innerL < -1.0e8f) || (innerR > 1.0e8f) ||
            ((innerR - innerL) < (XSEC_GAP_FRAC * cw)))
        {
            /* Either side had nothing at all, or the two sides were not far
             * enough apart. Both mean there is no hole, so there is no
             * crossing - and without the first test a group with material on
             * neither side would sail through on a gap of two billion pixels. */
            continue;
        }

        if (cover > bestCover)
        {
            bestCover = cover;
            bestY     = gy;
            bestW     = cw;
            bestN     = cnt;
            bestL     = hasL;
            bestR     = hasR;
            bestSpan  = span;
            bestGap   = innerR - innerL;
        }
    }

    /* ---------------------------------------------------------------
     * 3. Is it a crossing?
     *
     * The heading tests are the safety interlock, and the reason a hairpin can
     * never be mistaken for one. A tight corner also puts a line sideways in
     * the frame, and a tight corner is precisely where carrying on straight at
     * full power would be unrecoverable - so the road through the bars has to
     * be near enough straight before any of this is believed.
     * -------------------------------------------------------------*/
    /*
     * Three ways to see the same thing, and the car believes any of them.
     *
     * Which one shows up is the camera's decision, not the track's - it depends
     * on how the tracker chooses to cut the paint into vectors where our line
     * runs into the crossing's:
     *
     *   it follows the corner round      -> our own vectors bend outward and
     *                                       splay apart, and there are no loose
     *                                       bars at all
     *   it breaks the paint at the       -> the crossing's edges arrive as loose
     *   corner and drops the joint          bars with a track-width hole
     *   it breaks the paint but keeps    -> the two pieces still share an
     *   both pieces                         endpoint: a right-angle elbow, one
     *                                       either side, arms pointing apart
     *
     * All three are the same junction. A detector that knew only one of them
     * would go blank on a frame that plainly shows a crossing, which is worse
     * than useless: the lift and then the failsafe stop that this whole module
     * exists to prevent happen precisely on the frames it went blank for.
     */
    s.diverge = 0.0f;
    s.square  = 0.0f;
    if (m->haveTrack &&
        (absf(m->headFar) <= XSEC_MAX_HEAD) &&
        (absf(m->headNear) <= XSEC_MAX_HEAD) &&
        (absf(m->curv) <= XSEC_MAX_CURV))
    {
        if (diverging(segs, n, m, &s.diverge))
        {
            recog = true;
        }
    }
    if (m->haveTrack &&
        (absf(m->headFar) <= XSEC_ELB_MAX_HEAD) &&
        (absf(m->headNear) <= XSEC_ELB_MAX_HEAD) &&
        (absf(m->curv) <= XSEC_MAX_CURV))
    {
        if (elbowed(segs, n, m, &s.square))
        {
            recog = true;
        }
    }

    if ((bestN >= (uint8_t)XSEC_MIN_BARS) && (bestW > 0.0f) &&
        (bestCover >= (XSEC_MIN_COVER * bestW)) &&
        (bestY <= XSEC_BAR_NEAR_Y) &&
        (absf(m->headFar) <= XSEC_MAX_HEAD) &&
        (absf(m->headNear) <= XSEC_MAX_HEAD) &&
        (absf(m->curv) <= XSEC_MAX_CURV) &&
        bestL && bestR)
    {
        /*
         * Bars reaching out on BOTH sides of the corridor, and more than one of
         * them. That pair is the actual signature of a crossing - the two ends
         * of the other track's edge, cut in half by ours - and demanding it is
         * what keeps corners out.
         *
         * A single flat vector lying across the corridor is NOT enough, however
         * much it looks like a start line. Near the top of the frame a track
         * edge in a corner flattens out and reads exactly like one, and
         * accepting that fired 28 times a lap on clean circuits with no
         * junction anywhere on them. A start/finish line goes unrecognised as a
         * result, which costs nothing: track.c discards the bar either way, and
         * the car simply drives past it as it always did.
         */
        recog = true;
    }

    s.camAgreed = s_camUsed;
    s_camUsed   = false;
    s.recognised = recog;
    s.bars       = bestN;
    s.barY       = bestY;
    s.bothSides  = bestL && bestR;
    s.spanning   = bestSpan;
    s.cover      = (bestW > 0.0f) ? (bestCover / bestW) : 0.0f;
    s.gap        = (bestW > 0.0f) ? (bestGap / bestW) : 0.0f;

    if (recog)
    {
        if (s.agree < 255u)
        {
            s.agree++;
        }
        s_miss = 0u;
    }
    else
    {
        s.agree = 0u;
        if (s_miss < 255u)
        {
            s_miss++;
        }
    }

    s.runM += (travelM > 0.0f) ? travelM : 0.0f;
    if (recog)
    {
        s.seenM = 0.0f;
    }
    else
    {
        s.seenM += (travelM > 0.0f) ? travelM : 0.0f;
    }

    /* ---------------------------------------------------------------
     * 4. What to do about it.
     * -------------------------------------------------------------*/
    switch (s.phase)
    {
        case XSEC_IDLE:
            if (s.agree >= (uint8_t)XSEC_CONFIRM_FRAMES)
            {
                s.phase = XSEC_AHEAD;
                s.runM  = 0.0f;
            }
            else if ((s.seenM <= XSEC_MEMORY_M) && !m->haveTrack)
            {
                /*
                 * Recognised a crossing a moment ago, and the track has just
                 * gone. That is the junction arriving, and it is worth catching
                 * here as well as from the approach state.
                 *
                 * The sighting and the loss of track land almost together: the
                 * crossing's edges leave the sides of the frame in the last
                 * stride, and one frame later our own lines stop too. Whether
                 * the car is still formally approaching when that happens comes
                 * down to which of the two lands first, which is a coin toss and
                 * not something to hang the behaviour on. Distance since the
                 * last sighting is the honest test, and it is bounded.
                 */
                s.phase = XSEC_CROSSING;
                latch(steerNow, headNow);
            }
            else
            {
                /* nothing to do */
            }
            break;

        case XSEC_AHEAD:
            /* Latch either when the bars have come down close, or when the
             * corridor has already fallen apart underneath us - on a crossing
             * wider than our own track, the second happens first.
             *
             * But never while the road under the bumper is already turning.
             * See XSEC_LATCH_MAX_HEAD: that is the one cue a crossing cannot
             * silence, and committing against it is how a junction sitting in a
             * bend takes a car off the track. */
            if (((recog && (bestY >= XSEC_LATCH_Y)) || !m->haveTrack ||
                 (m->nValid < 3u)) &&
                (absf(m->headNear) <= XSEC_LATCH_MAX_HEAD))
            {
                s.phase = XSEC_CROSSING;
                latch(steerNow, headNow);
            }
            else if ((s.seenM > XSEC_MEMORY_M) && m->haveTrack &&
                     m->bothEdges && (m->nValid >= (uint8_t)SPEED_SEE_ROWS_FULL))
            {
                /*
                 * Give up only when the corridor is plainly healthy again -
                 * whatever that was, it was not a crossing.
                 *
                 * Not on the sighting going quiet by itself. In the last stride
                 * before a junction the crossing's edges slide off the sides of
                 * the frame, so the very thing that identified it disappears
                 * exactly when the car is about to need the answer. Dropping out
                 * there put the car back on the ordinary rules one frame before
                 * losing the track, and it lifted for a junction it had already
                 * correctly recognised.
                 *
                 * And not after a couple of quiet frames either, but after a
                 * distance. A real camera shows the corner at the top of the
                 * frame from two metres out and then, as the car closes, draws
                 * nothing along the crossing edge at all for a stretch - the
                 * third PixyMon frame - while the corridor still looks
                 * ordinary. Two blank frames there is thirty milliseconds, and
                 * a recognition thrown away at that point has to be made all
                 * over again from a frame that no longer shows the signature.
                 */
                s.phase = XSEC_IDLE;
                s.runM  = 0.0f;
            }
            else if (s.runM > XSEC_AHEAD_MAX_M)
            {
                /* Bound it anyway. Something recognised this long ago and never
                 * arrived was not a crossing. */
                s.phase = XSEC_IDLE;
                s.runM  = 0.0f;
            }
            else
            {
                /* still approaching */
            }
            break;

        case XSEC_CROSSING:
            /*
             * Get out at the first opportunity, and never later than the
             * distance cap.
             *
             * The held angle is an assumption - that the road carries on the way
             * it was going - and it is only true for as long as the road does.
             * On a bend the car can be a frame or two into the junction before
             * the turn has developed enough to steer for, so what gets frozen is
             * closer to straight than the road really is, and every extra
             * centimetre held is error that compounds. Waiting for a confident
             * two-sided corridor before letting go was enough, on a 1.2 m
             * radius, to put the car off the track; a usable corridor is enough
             * to steer by and is back sooner. The cap stays as the backstop, and
             * as the bound on how far a false positive can carry the car.
             */
            if ((s.runM >= XSEC_HOLD_M) ||
                (!recog && m->haveTrack && (m->nValid >= (uint8_t)XSEC_RESUME_ROWS)))
            {
                s.phase = XSEC_CLEAR;
                s.runM  = 0.0f;
            }
            else
            {
                /* still in it */
            }
            break;

        case XSEC_CLEAR:
        default:
            if (s.runM >= XSEC_CLEAR_M)
            {
                s.phase = XSEC_IDLE;
                s.runM  = 0.0f;
                s.agree = 0u;
            }
            break;
    }

    /*
     * What to steer while driving blind through it.
     *
     * The held angle is right for as long as the reason for it lasts. On a
     * crossing laid on a bend the reason is the bend, which is still there on
     * the far side, so the angle is held as it is. On a car that arrived
     * yawed - straightening up out of the corner before, or after a bump -
     * the reason is the yaw, and the yaw is gone a few tenths of a metre in;
     * an angle that was undoing it keeps turning the car after that, and on
     * a 17 cm wheelbase a modest correction held across a 60 cm junction
     * turns the car through most of a right angle. That is the car steering
     * off onto the other track. So the yaw share of the angle, read off the
     * near heading at the latch, is let go of over XSEC_HOLD_DECAY_M, and the
     * rest is kept.
     */
    if (s.phase == XSEC_CROSSING)
    {
        float k = (XSEC_HOLD_DECAY_M > 0.01f) ? (s.runM / XSEC_HOLD_DECAY_M) : 1.0f;

        if (k > 1.0f)
        {
            k = 1.0f;
        }
        s.steerOut = s.holdSteer * (1.0f - (s.holdYaw * k));
    }
    else
    {
        s.steerOut = s.holdSteer;
    }
#else
    (void)segs;
    (void)n;
    (void)m;
    (void)travelM;
    (void)steerNow;
    (void)headNow;
    s.phase      = XSEC_IDLE;
    s.recognised = false;
#endif

    *out = s;
}
