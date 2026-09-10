#include "recover.h"
#include "race_config.h"
#include <math.h>

static RecoverState s_r;

static void publish(RecoverState *out)
{
    *out = s_r;
}

void Recover_Init(void)
{
    s_r.phase  = RCV_IDLE;
    s_r.active = false;
    s_r.side   = 0.0f;
    s_r.nudge  = 0.0f;
    s_r.lean   = 0.0f;
    s_r.blind  = 0u;
    s_r.held   = 0u;
    s_r.probes = 0u;
    s_r.found  = 0u;
}

#if RCV_ENABLE

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
 * Which black line is missing?
 *
 * +1 when the unseen one is to the right, -1 when it is to the left, and 0 when
 * the answer is not clean. Not clean means either nothing was measured at all,
 * or some rows measured the left edge while others measured the right - and a
 * corridor assembled from two different pieces of evidence is not something to
 * go probing on, because there is no single side to probe toward.
 */
static float missing_side(const TrackModel *m)
{
    bool    anyL = false, anyR = false;
    uint8_t i;

    for (i = 0u; i < m->nValid; i++)
    {
        if (m->sawL[i])
        {
            anyL = true;
        }
        if (m->sawR[i])
        {
            anyR = true;
        }
    }

    if (anyL == anyR)
    {
        return 0.0f;
    }
    return anyL ? 1.0f : -1.0f;
}

void Recover_Update(const TrackModel *m, bool busy, bool pinched,
                    RecoverState *out)
{
    float side     = missing_side(m);
    bool  oneSided = m->haveTrack && !m->bothEdges && (side != 0.0f) &&
                     (m->nValid >= (uint8_t)RCV_MIN_ROWS);

    /*
     * Is the road in front of the bumper straight?
     *
     * This is the whole safety of the feature. A tight corner also arrives with
     * one edge missing - the outside line swings out of the frame - and there
     * the correct answer is the opposite of this one. What separates the two is
     * that a corner bends the near rows as well, and the near rows are the part
     * of the picture a narrow view cannot take away. If they are bent, the car
     * is in a corner and this module has nothing useful to say.
     */
    bool quiet = (fabsf(m->headNear) < RCV_MAX_HEAD);

    if (oneSided)
    {
        if (s_r.blind < 0xFFFFu)
        {
            s_r.blind++;
        }
    }
    else
    {
        s_r.blind = 0u;
    }

    if (s_r.held < 0xFFFFu)
    {
        s_r.held++;
    }

    switch (s_r.phase)
    {
        case RCV_PROBE:
            if (!oneSided)
            {
                /* Either the missing line came back, which is the point, or the
                 * track went away entirely, which is the failsafe's problem and
                 * not ours. Either way the probe is over. */
                if (m->haveTrack && m->bothEdges)
                {
                    s_r.found++;
                }
                s_r.phase = RCV_IDLE;
                s_r.held  = 0u;
            }
            else if (busy || pinched || !quiet ||
                     (s_r.held >= (uint16_t)RCV_MAX_FRAMES))
            {
                /* Out of time, or the road has started to bend after all. Stand
                 * down and let the ordinary logic drive the corner. */
                s_r.phase = RCV_BACKOFF;
                s_r.held  = 0u;
            }
            else
            {
                /* still looking */
            }
            break;

        case RCV_BACKOFF:
            /*
             * Waiting out a probe that did not work. Both conditions matter: the
             * timer stops an immediate retry, and requiring the corridor to be
             * measured on both sides again stops the car probing a second time
             * inside the same hairpin, which would be a limit cycle rather than
             * a recovery.
             */
            if (!oneSided && (s_r.held >= (uint16_t)RCV_COOL_FRAMES))
            {
                s_r.phase = RCV_IDLE;
                s_r.held  = 0u;
            }
            break;

        default:
            if (oneSided && quiet && !busy && !pinched &&
                (s_r.blind >= (uint16_t)RCV_ARM_FRAMES))
            {
                s_r.phase = RCV_PROBE;
                s_r.held  = 0u;
                s_r.side  = side;
                if (s_r.probes < 0xFFFFu)
                {
                    s_r.probes++;
                }
            }
            break;
    }

    s_r.active = (s_r.phase == RCV_PROBE);

    /*
     * The ramp is what makes the first part of the probe a straighten and only
     * the later part a lean. Starting at zero means the car gives up the racing
     * line before it gives up the centre of the corridor, so if the missing line
     * reappears immediately - which is the common case - nothing else moved.
     */
    s_r.nudge = s_r.active
                    ? clampf((float)s_r.held / (float)RCV_RAMP_FRAMES, 0.0f, 1.0f)
                    : 0.0f;

    /*
     * ...and whether the lean half of it is allowed.
     *
     * side * headFar < 0 says the line that went missing is on the far side of
     * a bend the car can already see - the outside of the corner. Two reasons
     * not to lean toward it. It is the direction that takes a car off a corner,
     * and it would not work anyway: the outside line is out of frame because of
     * where the corner is, not because of where the car is, so a few degrees of
     * yaw will not bring it back. Straightening still earns its keep, since the
     * racing line being given up was drawn from one edge and a guess.
     */
    if (s_r.active && (s_r.side * m->headFar < 0.0f) &&
        (fabsf(m->headFar) > RCV_BEND_HEAD))
    {
        s_r.lean = 0.0f;
    }
    else
    {
        s_r.lean = s_r.side * s_r.nudge;
    }

    publish(out);
}

#else /* RCV_ENABLE */

void Recover_Update(const TrackModel *m, bool busy, bool pinched,
                    RecoverState *out)
{
    (void)m;
    (void)busy;
    (void)pinched;
    Recover_Init();
    publish(out);
}

#endif /* RCV_ENABLE */
