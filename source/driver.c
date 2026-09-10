#include "driver.h"
#include "race_config.h"
#include "speed_ctl.h"
#include "intersection.h"
#include "recover.h"
#include <math.h>

static DriveState s_st;

static float s_steer;     /* what the servo is being given right now  */
static float s_steerTgt;  /* what the controller wants                */
static float s_errF;      /* low pass filtered aim error              */
static float s_errFPrev;
static float s_vTarget;   /* speed the planner asked for              */
static float s_speedCmd;  /* speed after the acceleration ramp        */
static float s_speedFrac; /* 0..1, feeds the look ahead distance      */
static float s_sinceFrame;
static float s_blindMs;
static float s_blindM;    /* metres travelled since the track was last seen */
static float s_frameDt;     /* smoothed gap between camera frames, seconds */
static float s_frameDtBest; /* the best this camera has managed - learned      */
static float s_brakeMs;
static float s_brakeMag;
static float s_xsecM;     /* metres driven since the last camera frame */
static float s_steerHold; /* steering to freeze if a crossing turns up  */
static float s_headHold;  /* the near heading behind it, same filter    */
static float s_xsecV;     /* speed frozen on the way into a crossing    */
static uint8_t s_chordFrames; /* consecutive frames with no bend measurable */

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
 * Deadband with no step in it. Inside the band the output is zero, outside it the
 * full range is still reachable. A hard deadband would make the servo jump by the
 * band width the moment it is crossed, which shows up as a twitch on every straight.
 */
static float soft_deadband(float x, float d)
{
    if (d <= 0.0f)
    {
        return x;
    }
    if (d >= 0.999f)
    {
        return 0.0f;
    }
    if (x > d)
    {
        return (x - d) / (1.0f - d);
    }
    if (x < -d)
    {
        return (x + d) / (1.0f - d);
    }
    return 0.0f;
}

/*
 * Is this a corner, or just the road not being perfectly straight?
 *
 * 0 means a kink not worth reacting to, 1 means a corner that deserves the full
 * treatment. Everything below CHICANE_MAX_HEAD scores zero, which is what lets the
 * car drive straight through small chicanes AND keep its speed up while doing it.
 * Both the steering deadband and the speed planner run off this one number, so the
 * car never ends up lifting for something it decided not to steer for.
 */
static float ramp01(float v, float ignore, float full)
{
    float span = full - ignore;

    if (span < 0.05f)
    {
        span = 0.05f;
    }
    return clampf((fabsf(v) - ignore) / span, 0.0f, 1.0f);
}

/*
 * Speed to hold through a recognised crossing.
 *
 * Not simply flat out. Inside the junction there is nothing left to plan from,
 * so the right answer is the one the car had already arrived at on the approach
 * - which on a straight IS flat out, because nothing was slowing it, and on a
 * bend is the speed that bend was asking for. Commanding maximum regardless
 * would mean a junction sitting in a corner cancelled the braking for the
 * corner, which is a good way to lose a car that was about to turn.
 *
 * Floored so that a momentary dip on the way in cannot leave the car crawling,
 * or stopped, in the middle of a junction - the failure this whole module
 * exists to prevent.
 */
static float xsec_hold_speed(void)
{
    float v = s_xsecV;

#if XSEC_FULL_POWER
    /*
     * Not the speed the approach settled on - all of it.
     *
     * Inside the latch there is no corridor to be careful about: our lines are
     * not painted here, so every cue that would normally meter the throttle is
     * describing a road that does not exist for the next half metre. The car is
     * already driving on a held angle and a bounded distance, and both of those
     * were committed to on a road three cues agreed was straight. Carrying a
     * lift through as well buys nothing - there is nothing left for it to buy
     * safety from - and costs the 348 ms the motor takes to give the speed back.
     */
    v = SPEED_MAX;
#endif

    if (v < SPEED_MIN)
    {
        v = SPEED_MIN;
    }
    return clampf(v * XSEC_SPEED_FRAC, 0.0f, 100.0f);
}

static float cornerness_of(const TrackModel *m)
{
    float a = ramp01(m->headFar, CORNER_HEAD_IGNORE, CORNER_HEAD_FULL);
    float b = ramp01(m->curv, CORNER_CURV_IGNORE, CORNER_CURV_FULL);

    return (a > b) ? a : b;
}

void Driver_Init(void)
{
    uint8_t i;

    Track_Init();
    RL_Init();
    SpeedCtl_Init();
    Xsec_Init();
    Recover_Init();

    s_steer      = 0.0f;
    s_steerTgt   = 0.0f;
    s_errF       = 0.0f;
    s_errFPrev   = 0.0f;
    s_vTarget    = 0.0f;
    s_speedCmd   = 0.0f;
    s_speedFrac  = 0.0f;
    s_chordFrames = 0u;
    s_sinceFrame  = 0.0f;
    s_blindMs     = 0.0f;
    s_blindM      = 0.0f;
    s_frameDt     = 0.020f;
    s_frameDtBest = 0.020f;
    s_brakeMs     = 0.0f;
    s_brakeMag    = 0.0f;
    s_xsecM       = 0.0f;
    s_steerHold   = 0.0f;
    s_headHold    = 0.0f;
    s_xsecV       = 0.0f;

    s_st.severity   = 0.0f;
    s_st.steerTgt   = 0.0f;
    s_st.lostFrames = 0u;
    s_st.frames     = 0u;
    s_st.elapsedMs  = 0.0f;
    s_st.exiting    = false;

    s_st.track.haveTrack = false;
    s_st.track.nValid    = 0u;
    s_st.track.topRow    = 0u;
    s_st.track.bothEdges = false;
    s_st.track.headNear  = 0.0f;
    s_st.track.headFar   = 0.0f;
    s_st.track.curv      = 0.0f;
    for (i = 0u; i < TRK_ROWS; i++)
    {
        s_st.track.valid[i] = false;
    }

    s_st.line.targetX = CAM_CENTER_X;
    s_st.line.laRow   = 0u;
    s_st.line.bias    = 0.0f;
    s_st.line.chicane = false;
    s_st.line.clamped = false;
    s_st.line.probe   = 0.0f;

    /* Seed the caller-visible copy. Safe only now the track model above has
     * been cleared, since the detector reads it. */
    Xsec_Update((const TrkSegment *)0, 0u, &s_st.track, 0.0f, 0.0f, 0.0f, &s_st.xsec);
    Recover_Update(&s_st.track, false, false, &s_st.rcv);
}

const DriveState *Driver_State(void)
{
    return &s_st;
}

/* ---- steering ----------------------------------------------------------- */

static void plan_steering(float dtFrame)
{
    const TrackModel *m = &s_st.track;
    const RacingLine *l = &s_st.line;
    float             errN, headN, uPos, db, dTerm, steer;

    /* How far the aim point sits beside the car, as a fraction of half the image. */
    errN = (l->targetX - CAM_CENTER_X) / (0.5f * (float)PIXY_LINE_W);

    /* Which way the road points right in front of the bumper. Without this the car
     * only reacts to being off line, and always turns in a fraction too late. */
    headN = clampf(m->headNear / STEER_HEAD_SCALE, -1.5f, 1.5f);

    /*
     * How corner-like the road is, 0 for a kink and 1 for a proper corner. The
     * deadband slides with it: wide when there is nothing worth turning for, narrow
     * once there is. That is what makes the car drive straight through a small
     * chicane instead of weaving down it.
     */
    {
        float cornerness = cornerness_of(m);

        db = CHICANE_DEADBAND_BIG + ((CHICANE_DEADBAND - CHICANE_DEADBAND_BIG) * cornerness);

        /* A recognised chicane gets the wide band whatever the headings say. */
        if (l->chicane)
        {
            db = CHICANE_DEADBAND_BIG;
        }
    }

    /* ...but never when the safety check had to pull the aim point back. Then the
     * wiggle is not a wiggle, it is a black line, and the car steers. */
    if (l->clamped)
    {
        db = 0.0f;
    }

#if STEER_PURE_PURSUIT
    /*
     * Geometric steering: the angle that actually puts the car on the aim point,
     * rather than a fixed gain on the pixel error.
     *
     *     steer = K * Dx * w      K = 200*(L/W) / (f^2 * maxSteer)
     *
     * Dx is how far the aim point sits beside the car in pixels, and w is the
     * corridor width in pixels at that row - which is the firmware's own
     * calibration-free measure of how far away the row is. Multiplying by it is
     * what turns a fixed gain into one that schedules itself with look-ahead.
     */
    {
        float w = m->width[l->laRow];
        float dx = l->targetX - CAM_CENTER_X;

        /* A row the model is unsure about can carry a stale or silly width, and
         * width multiplies the command directly. */
        if (w < TRK_MIN_ROW_WIDTH_PX)
        {
            w = TRK_MIN_ROW_WIDTH_PX;
        }
        if (w > TRK_WIDTH_NEAR_PX)
        {
            w = TRK_WIDTH_NEAR_PX;
        }

        uPos = ((STEER_PP_SCALE * STEER_PP_K * dx * w) + (STEER_KH * headN)) / 100.0f;
    }
#else
    uPos = ((STEER_KP * errN) + (STEER_KH * headN)) / 100.0f;
#endif
    uPos = soft_deadband(uPos, db);

    /* Damping runs on the filtered aim error. Feeding it from the post-deadband
     * demand instead was tried and is worse: the rescaling at the edge of the band
     * makes the rate term spike exactly when the car is deciding whether a bend
     * matters, and it destabilises real S bends. */
    s_errF += STEER_D_ALPHA * (errN - s_errF);
    dTerm = (dtFrame > 1e-4f) ? (STEER_KD * ((s_errF - s_errFPrev) / dtFrame)) : 0.0f;
    s_errFPrev = s_errF;

    steer = (uPos * 100.0f) + dTerm;

    /*
     * Curvature feedforward.
     *
     * A constant-radius corner needs a constant steering angle, L/R. Everything
     * above is proportional to an error, and proportional terms can only hold a
     * constant output by holding a constant error - so without this the car has
     * to sit off the line for the whole corner just to keep the wheels turned.
     * On a 90 cm corner that standing offset is wider than half the track, which
     * is precisely why a P-only car cannot hold an apex.
     *
     * curv is headFar - headNear, which cancels the car's own lateral offset and
     * leaves the bend itself, and it is proportional to real curvature. So hand
     * the servo the angle the geometry needs and let the error terms go back to
     * doing what they are for.
     *
     * Scaled by cornerness, which is exactly zero on a straight, and switched
     * off in a recognised chicane: the car has already decided to drive that one
     * straight through, and feeding forward into it would undo the decision.
     */
    if (!l->chicane)
    {
        /*
         * curv is only a curvature while the road bends one way. In an S the two
         * headings point opposite ways and their difference is large without any
         * single radius existing at all - which is why plan_speed already throws
         * curv away for a chicane. Feeding that number forward asks for most of
         * full lock on a road that is close to straight.
         *
         * So the term is clamped to a curvature the track can really contain.
         * STEER_KFF_CURV_MAX is 2.0, a half-metre radius, tighter than anything
         * an NXP Cup layout puts down.
         */
        float c = clampf(m->curv, -STEER_KFF_CURV_MAX, STEER_KFF_CURV_MAX);

        steer += STEER_KFF * c * cornerness_of(m);
    }

    /* Left and right are not mechanically identical on this car. */
    steer *= (steer >= 0.0f) ? STEER_GAIN_RIGHT : STEER_GAIN_LEFT;

    /*
     * Driving through a recognised crossing.
     *
     * Our own two lines are missing here and what is left in the frame is the
     * crossing track, so everything computed above is being computed from the
     * wrong road. Hold the angle the car came in on instead. On a square
     * crossing that is straight ahead; on a crossing laid on a bend it is the
     * radius the car was already turning, which is exactly what should be
     * carried across a gap this short.
     */
    if (Xsec_Holding(&s_st.xsec))
    {
        steer = s_st.xsec.steerOut;
    }

    s_steerTgt    = clampf(steer, STEER_LIMIT_LEFT, STEER_LIMIT_RIGHT);
    s_st.steerTgt = s_steerTgt;
}

/* ---- speed -------------------------------------------------------------- */

static float see_factor(uint8_t rows)
{
    if (rows >= (uint8_t)SPEED_SEE_ROWS_FULL)
    {
        return 1.0f;
    }
    if (rows <= (uint8_t)SPEED_SEE_ROWS_MIN)
    {
        return SPEED_SEE_FLOOR;
    }
    {
        float span = (float)(SPEED_SEE_ROWS_FULL - SPEED_SEE_ROWS_MIN);
        float t    = (float)(rows - (uint8_t)SPEED_SEE_ROWS_MIN) / span;
        return SPEED_SEE_FLOOR + ((1.0f - SPEED_SEE_FLOOR) * t);
    }
}

static void plan_speed(void)
{
    const TrackModel *m = &s_st.track;
    float             hf, cv, st, sev, v;

    if (s_st.line.chorded)
    {
        if (s_chordFrames < 255u)
        {
            s_chordFrames++;
        }
    }
    else
    {
        s_chordFrames = 0u;
    }

    /* Same "is this really a corner" test the steering uses, so the car never throws
     * away speed for a bend it has already decided to drive straight through. */
    hf = ramp01(m->headFar, CORNER_HEAD_IGNORE, CORNER_HEAD_FULL);
    cv = ramp01(m->curv, CORNER_CURV_IGNORE, CORNER_CURV_FULL);
    st = fabsf(s_steerTgt) / 100.0f;

    /* A small chicane is not a corner. Without this the curvature term reads the S
     * bend as a huge change of direction and throws away all the speed. */
    if (s_st.line.chicane)
    {
        cv = 0.0f;
    }

    /* The largest cue wins. Because one of them is the far heading, a corner that is
     * still only visible in the distance already slows the car - that is the braking
     * point, and it lands before the corner instead of inside it. */
    sev = SPEED_W_HEAD_FAR * hf;
    if ((SPEED_W_CURV * cv) > sev)
    {
        sev = SPEED_W_CURV * cv;
    }
    if ((SPEED_W_STEER * st) > sev)
    {
        sev = SPEED_W_STEER * st;
    }

    sev           = clampf(sev / SPEED_SEVERITY_FULL, 0.0f, 1.0f);
    s_st.severity = sev;

    v = SPEED_MAX - ((SPEED_MAX - SPEED_MIN) * sev);

    /*
     * Inside a recognised crossing there is no useful corridor at all - our own
     * lines are simply not painted here - so the whole plan above is describing
     * the wrong road and the held one replaces it outright.
     */
    if (Xsec_Holding(&s_st.xsec))
    {
        s_vTarget    = xsec_hold_speed();
        s_st.exiting = true;
        return;
    }

    /*
     * Approaching one is different, and the difference matters. The corner cue
     * computed above keeps its vote: a bend is still a bend whether or not a
     * junction happens to sit in it, and suppressing that for the whole second
     * it takes to arrive would carry the car into the bend with the power still
     * on. What is stood down is only the three cues a crossing actually
     * corrupts - the corridor is short because our lines stop at the junction
     * rather than because the car cannot see, and one edge is missing because
     * the other is under the crossing track rather than because it was lost.
     */
    if (!Xsec_KeepPower(&s_st.xsec) || (sev > XSEC_KEEP_POWER_SEV))
    {
        /* Never drive faster than the distance that can actually be seen. */
        v *= see_factor(m->nValid);

        /* One edge visible means the other is a guess. Guesses get less speed. */
        if (!m->bothEdges)
        {
            v *= SPEED_ONE_EDGE_CAP;
        }

        /* The safety check had to pull the aim point back, so the corridor is
         * tighter than the racing line wanted. Take a little more out. */
        if (s_st.line.clamped)
        {
            v *= 0.90f;
        }
    }
#if XSEC_FULL_POWER_AHEAD
    else
    {
        /*
         * A crossing recognised ahead, and the corner cue quiet.
         *
         * The branch above has just been skipped because the three cues it
         * applies are the three a junction corrupts, so what is left in v is the
         * corner plan alone - and the corner plan has already been asked, in the
         * condition on that branch, whether there is a bend coming. It said no.
         * The remaining difference between v and SPEED_MAX is severity the car
         * has scored for a road that is straight, so the car goes.
         *
         * The gate is doing the work, not this line: the moment sev climbs past
         * XSEC_KEEP_POWER_SEV the whole branch swaps over, the ordinary rules
         * come back with the final say, and the junction waits its turn.
         *
         * Off by default, and this is why. The branch is only ever reached
         * while a crossing is recognised AHEAD - once the car commits,
         * plan_speed has already returned through xsec_hold_speed above - so
         * everything it does rests on a recognition that has not been acted on
         * yet and may be wrong. It is: rendered the way a real camera draws a
         * frame, with each straight piece of paint one long vector, the bar
         * test reads the top two rows of an ordinary corner as a crossing 62
         * times over the five clean circuits, and each of those put the car to
         * SPEED_MAX for as long as the corner cue stayed quiet. The car surges,
         * the corner arrives, severity climbs, the branch swaps back and it
         * brakes. Surge, brake, surge is what that feels like from outside.
         *
         * With this off a false approach costs nothing at all: the car simply
         * does not lift for a corridor that has gone short, which is the whole
         * conservative point of the feature, and a crossing it actually
         * commits to still gets SPEED_MAX from xsec_hold_speed for the bounded
         * distance of the latch.
         */
        v = SPEED_MAX;
    }
#endif

    /*
     * Full power is for a corner the camera actually described.
     *
     * Everything the plan above is built on - headFar, curv, the steering command -
     * comes out of this frame's vectors, so when there are too few of them to carry
     * curvature the cues do not report a corner at all: a bend merged into one chord
     * per edge arrives as a straight at an angle, scores no severity, and is driven
     * at the speed of the straight it is pretending to be. No corner cue can catch
     * that, because the cues are the thing that has gone blind. track.c's canCurve
     * can, because it is a statement about the description rather than the road: it
     * says whether any edge was described by more than one vector, which is what
     * makes curv a measurement rather than an arithmetic zero.
     *
     * This sits outside the crossing test above on purpose. A junction is a reason
     * for the corridor to be short and one-sided, which is why those three cues are
     * stood down there; it is not a reason for a bend to arrive with no curvature in
     * it, and if one does, the power still comes down. It costs a little at a
     * junction that sits on a bend, which -xsec measures.
     *
     * It does not fire on a straight, it does not fire on a corner the camera broke
     * into enough pieces to measure, and being a ceiling rather than a cut it does
     * nothing to a corner the cues have already slowed below it. What is left is the
     * one case it is for: the power of a straight, about to be used on a bend.
     */
    if ((s_chordFrames >= (uint8_t)SPEED_CHORD_FRAMES) && (v > SPEED_CHORD_CEIL))
    {
        v = SPEED_CHORD_CEIL;
    }

    /* A camera that has slowed down gets a car that has slowed down. Every correction
     * now arrives later, so the same speed buys less safety margin. */
    if (s_frameDt > s_frameDtBest)
    {
        v *= clampf(s_frameDtBest / s_frameDt, CAM_RATE_FLOOR, 1.0f);
    }

    v *= SPEED_SCALE;

    s_vTarget = clampf(v, 0.0f, 100.0f);

    /* Corner exit: still turning here, straight from here on. Time for full power. */
    s_st.exiting = (fabsf(m->headFar) < (0.30f * LINE_HEAD_REF)) &&
                   (fabsf(m->headNear) > (0.30f * LINE_HEAD_REF));
}

/* ---- main step ---------------------------------------------------------- */

void Driver_Step(bool freshFrame, const TrkSegment *segs, uint8_t n, float dt, DriveCmd *cmd)
{
    float v;
    float base, diff, inner, outer, outL, outR, m1, m2;
    bool  running;

    dt = clampf(dt, 1.0e-4f, 0.2f);

    s_st.elapsedMs += dt * 1000.0f;
    s_sinceFrame += dt;

    /* ---- planning, only when there is something new to look at ---- */
    if (freshFrame)
    {
        s_blindMs = 0.0f;
        s_st.frames++;

        /* Learn how fast this camera normally is, so a drop in rate can be spotted
         * without hard-coding what "normal" looks like. Quick to accept a better
         * rate, very slow to forget one. */
        {
            float gap = clampf(s_sinceFrame, 1.0e-4f, 0.5f);

            s_frameDt += 0.25f * (gap - s_frameDt);
            if (s_frameDt < s_frameDtBest)
            {
                s_frameDtBest = s_frameDt;
            }
            else
            {
                s_frameDtBest += 0.0008f * (s_frameDt - s_frameDtBest);
            }
        }

        {
            /*
             * The track model is built from what the crossing detector, on its
             * previous verdict, says is track: everything, unless a junction is
             * in play, in which case the crossing's own edges are kept out of
             * it. See Xsec_ForTrack.
             */
            TrkSegment trk[XSEC_MAX_SEGS];
            uint8_t    nt = Xsec_ForTrack(&s_st.xsec, segs, n, trk);
            bool       haveTrack = Track_Update(trk, nt, &s_st.track);

            /*
             * Look for a crossing before anything is planned, and do it whether
             * or not the track model came back usable. Losing the track IS what
             * a wide crossing looks like from here, so a detector that only ran
             * on good frames would go quiet exactly when it is needed.
             *
             * The angle handed over is not this frame's, and not last frame's
             * either. Both are already contaminated by the time the car commits:
             * the crossing eats the corridor a frame or two before the latch and
             * the steering reacts to that. s_steerHold is sampled only while the
             * corridor is still whole - before anything is recognised, and on
             * the approach for as long as both lines are in view and the model
             * runs deep. A crossing can be recognised two metres out, and an
             * angle frozen back there is a second stale by the time the car
             * commits: whatever the car did in that second, the wheel would
             * still be where it was before it.
             *
             * The near heading goes with it, filtered the same way, so that the
             * detector can tell an angle that was following a bend from one
             * that was straightening the car up.
             */
            XsecPhase was = s_st.xsec.phase;

            if ((was == XSEC_IDLE) ||
                ((was == XSEC_AHEAD) && s_st.track.haveTrack && s_st.track.bothEdges &&
                 (s_st.track.nValid >= (uint8_t)XSEC_HOLD_MIN_ROWS)))
            {
                s_steerHold += XSEC_HOLD_ALPHA * (s_steerTgt - s_steerHold);
                s_headHold += XSEC_HOLD_ALPHA * (s_st.track.headNear - s_headHold);
            }

            Xsec_Update(segs, n, &s_st.track, s_xsecM, s_steerHold, s_headHold, &s_st.xsec);
            s_xsecM = 0.0f;

            /* Committing. Freeze the speed the approach had settled on, which
             * already has the crossing's own effect on the corridor discounted
             * but still respects a corner the car is heading into. */
            if ((s_st.xsec.phase == XSEC_CROSSING) && (was != XSEC_CROSSING))
            {
                s_xsecV = s_vTarget;
            }

            /*
             * Then ask whether the car is driving on one black line and a
             * guess. Like the crossing detector this runs whether or not the
             * model came back usable, so its idea of how long the car has been
             * half blind does not depend on the frames it was fully blind.
             *
             * It stands down for anything the crossing detector has an opinion
             * about. A junction interrupts our lines, which is one of the ways
             * a corridor ends up one-sided, and there the answer is already
             * decided: hold the wheel where it was. Two modules steering for
             * the same missing paint would only fight.
             */
            Recover_Update(&s_st.track, (s_st.xsec.phase != XSEC_IDLE),
                           s_st.line.clamped, &s_st.rcv);

            if (haveTrack)
            {
                float dtFrame = clampf(s_sinceFrame, 1.0e-4f, 0.2f);

                s_st.lostFrames = 0u;
                RL_Compute(&s_st.track, s_speedFrac, &s_st.rcv, &s_st.line);
                plan_steering(dtFrame);
                plan_speed();
            }
            else if (Xsec_Holding(&s_st.xsec))
            {
                /* Mid-crossing. The corridor is gone because the paint is gone,
                 * which is not the same as being lost, so the failsafe counter
                 * is left alone and the held plan is simply re-asserted.
                 *
                 * The steering filter is deliberately not run: there is no real
                 * aim point this frame, and feeding it the leftovers would leave
                 * the damping term primed with nonsense for the frame the car
                 * comes out the other side. */
                s_steerTgt      = clampf(s_st.xsec.steerOut, STEER_LIMIT_LEFT,
                                         STEER_LIMIT_RIGHT);
                s_st.steerTgt   = s_steerTgt;
                s_vTarget       = xsec_hold_speed();
                s_st.exiting    = true;
                s_st.lostFrames = 0u;
            }
            else if (s_st.lostFrames < 0xFFFFu)
            {
                s_st.lostFrames++;
            }
            else
            {
                /* counter pinned, nothing more to do */
            }
        }

        s_sinceFrame = 0.0f;
    }
    else
    {
        s_blindMs += dt * 1000.0f;
    }

    running = (s_st.elapsedMs >= START_DELAY_MS);

    /* ---- start up ---- */
    v = s_vTarget;
    if (!running)
    {
        /* Wheels dead and straight while the car is being placed on the track. */
        v          = 0.0f;
        s_steerTgt = 0.0f;
        s_speedCmd = 0.0f;
    }
    else if (s_st.elapsedMs < (START_DELAY_MS + START_RAMP_MS))
    {
        v *= (s_st.elapsedMs - START_DELAY_MS) / START_RAMP_MS;
    }
    else
    {
        /* fully armed */
    }

    /* ---- losing the track ----
     * A frame or two with no line is normal, so the first few are coasted through on
     * the last known plan. After that the car slows, and if it really cannot find the
     * track it stops. Stopping loses a race; ploughing across the grass at full speed
     * loses the car. */
    /*
     * How far the car has travelled since it last saw the track.
     *
     * Distance, not a count of frames. What limits how wrong a stale plan can
     * get is the ground covered while believing it, and a frame is not a fixed
     * amount of ground - it is longer on a straight than in a hairpin, and
     * longer again if the camera slows down. Counting frames only looked right
     * while the motor lag stopped the car ever obeying the speed caps below;
     * once it obeys, the same thirty frames become thirty frames of crawling and
     * the car parks itself in the middle of a dropout it could have driven
     * through. Metres do not have that problem.
     */
    {
#if SPEED_CLOSED_LOOP
        float vNow = SpeedCtl_State()->vEstMs;
#else
        /* The open-loop path never runs the observer, so read the speed straight
         * off the static map instead. It ignores the motor lag and so runs a
         * little ahead of the truth, which errs toward stopping sooner. */
        float vNow = SpeedCtl_SpeedForDuty(s_speedCmd);
#endif
        /* Ground covered since the last camera frame, which is what the crossing
         * detector measures its own patience in. */
        s_xsecM += vNow * dt;

        if (s_st.lostFrames > 0u)
        {
            s_blindM += vNow * dt;
        }
        else
        {
            s_blindM = 0.0f;
        }
    }

    /*
     * Driving through a recognised crossing is not the same as having lost the
     * track, even though it looks identical from here: the lines really are
     * absent, and they are absent for a known and short distance. The detector
     * caps that distance itself, so the failsafe below is stood down rather than
     * being allowed to slow and then stop the car in the middle of a junction.
     */
    if (Xsec_Holding(&s_st.xsec))
    {
        s_blindM = 0.0f;
    }
    /* s_blindMs is deliberately left running. It measures frames not arriving at
     * all, which is a dead camera or a dead bus - still fatal in a junction. */

    if (running)
    {
        if (s_blindMs > CAM_TIMEOUT_MS)
        {
            v = 0.0f;
        }
        else if ((s_blindM > LOST_STOP_M) ||
                 ((s_speedCmd < 6.0f) &&
                  (s_st.lostFrames > (uint16_t)LOST_STOP_FRAMES)))
        {
            /* The frame count survives only as the backstop for the one case
             * distance cannot catch: a car that has already come to a halt
             * covers no more ground, so its distance budget never runs out and
             * it would sit there on the slow cap forever. Everywhere else the
             * frame count must NOT get a vote - the moment the caps below take
             * effect the car is crawling, and thirty frames of crawling is a
             * few centimetres, so a frame-based stop would park the car inside
             * every dropout it was perfectly able to drive through. */
            v = 0.0f;
        }
        else if (s_blindM > LOST_SLOW_M)
        {
            float cap = SPEED_LOST * 0.5f;
            if (v > cap)
            {
                v = cap;
            }
        }
        else if (s_blindM > LOST_COAST_M)
        {
            if (v > SPEED_LOST)
            {
                v = SPEED_LOST;
            }
        }
        else
        {
            /* brief dropout, carry on with the last plan */
        }
    }

    /* ---- brakes ----
     * Only worth doing from a real speed, and only once per braking event: the pulse
     * is latched, runs down, and by the time it ends the speed command has already
     * been dropped to the new target. */
#if BRAKE_ENABLE && !SPEED_CLOSED_LOOP
    if (running && (s_brakeMs <= 0.0f) && (s_speedCmd > SPEED_MIN) &&
        ((s_speedCmd - v) > BRAKE_TRIGGER))
    {
        float headroom = SPEED_MAX - SPEED_MIN - BRAKE_TRIGGER;
        float k;

        if (headroom < 1.0f)
        {
            headroom = 1.0f;
        }
        k = clampf(((s_speedCmd - v) - BRAKE_TRIGGER) / headroom, 0.0f, 1.0f);

        s_brakeMs  = BRAKE_MAX_MS * (0.35f + (0.65f * k));
        s_brakeMag = BRAKE_REVERSE_MAX * (0.40f + (0.60f * k));
    }
#endif

#if SPEED_CLOSED_LOOP
    /*
     * The planner's number is a speed, not a duty. Hand it to speed_ctl, which
     * inverts the measured duty->speed map and the 348 ms motor pole and returns
     * the duty that actually produces it.
     *
     * This also replaces the reverse brake pulse above. The pulse existed
     * because the old code could only drop the duty and wait; with the lag
     * inverted, a falling speed reference produces a proportionally negative
     * duty on its own - a metered brake instead of a fixed-length stab, and it
     * stops braking exactly when the speed is right rather than when a timer
     * expires.
     */
    if (!running)
    {
        SpeedCtl_Init();
        s_speedCmd = 0.0f;
    }
    else
    {
        bool allowBrake = (s_st.elapsedMs >= (START_DELAY_MS + START_RAMP_MS));

        float acc = s_st.exiting ? SPEED_ACC_EXIT_MS2 : SPEED_ACC_MS2;

        s_speedCmd = SpeedCtl_Step(v, dt, allowBrake, acc);
    }

    /* The look-ahead and the racing line slide on how fast the car IS, not on
     * what was asked for. Those differ by most of a second while the motor
     * catches up, and during that second the old code was choosing its aim point
     * for a speed the car had not reached. */
    s_speedFrac = SpeedCtl_SpeedFrac();
#else
    /* ---- speed ramp: ease on, drop instantly ---- */
    if (v > s_speedCmd)
    {
        float acc = s_st.exiting ? SPEED_ACCEL_EXIT_PER_S : SPEED_ACCEL_PER_S;

        s_speedCmd += acc * dt;
        if (s_speedCmd > v)
        {
            s_speedCmd = v;
        }
    }
    else
    {
        s_speedCmd = v;
    }

    s_speedFrac = clampf(s_speedCmd / SPEED_MAX, 0.0f, 1.0f);
#endif

    /* ---- servo rate limit ---- */
    {
        float step = STEER_SLEW_PER_S * dt;
        float d    = s_steerTgt - s_steer;

        if (d > step)
        {
            d = step;
        }
        if (d < -step)
        {
            d = -step;
        }
        s_steer += d;
        s_steer = clampf(s_steer, STEER_LIMIT_LEFT, STEER_LIMIT_RIGHT);
    }

    /* ---- outputs ---- */
    cmd->braking = false;
    base         = s_speedCmd;
    diff         = 0.0f;

    if (s_brakeMs > 0.0f)
    {
        s_brakeMs -= dt * 1000.0f;
        base         = -s_brakeMag;
        cmd->braking = true;
    }
    else
    {
        /* Torque vectoring: the wheels are split around the commanded speed rather
         * than by dragging the inside one down. Same difference between the wheels,
         * so the same yaw moment turning the car into the corner - but the average
         * of the two stays at what the speed planner actually asked for.
         *
         * Slowing only the inside wheel, which is the obvious way to do this, quietly
         * throws away up to a fifth of the drive at full lock. That is precisely the
         * moment the car most needs it. */
        diff = 0.5f * DIFF_GAIN * (fabsf(s_steer) / 100.0f);
        diff = clampf(diff, 0.0f, 0.45f);
    }

    /* What "braking" means under SPEED_CLOSED_LOOP.
     *
     * The reverse-pulse block above is compiled out whenever SPEED_CLOSED_LOOP is
     * set, so s_brakeMs never goes positive and this flag - and TLM_F_BRAKING with
     * it - could never be set at all. speed_ctl brakes by inverting the motor pole
     * instead, which shows up as a negative duty in base. Test that directly, and
     * before the differential and any MOTOR*_INVERT, so the meaning does not depend
     * on wiring config. */
    if (base < 0.0f)
    {
        cmd->braking = true;
    }

    outer = base * (1.0f + diff);
    inner = base * (1.0f - diff);

    /* If the outer wheel runs out of range, take the excess off both rather than
     * scaling them together - that keeps the yaw moment instead of shrinking it. */
    if (outer > 100.0f)
    {
        float over = outer - 100.0f;

        outer = 100.0f;
        inner -= over;
    }

    if (s_steer >= 0.0f) /* turning right, so the right wheel is on the inside */
    {
        outL = outer;
        outR = inner;
    }
    else
    {
        outL = inner;
        outR = outer;
    }

    m1 = outL;
    m2 = outR;

#if MOTOR_SWAP_SIDES
    {
        float t = m1;
        m1      = m2;
        m2      = t;
    }
#endif
#if MOTOR1_INVERT
    m1 = -m1;
#endif
#if MOTOR2_INVERT
    m2 = -m2;
#endif

    cmd->steer = s_steer;
    cmd->left  = clampf(m1, -100.0f, 100.0f);
    cmd->right = clampf(m2, -100.0f, 100.0f);
    cmd->speed = s_speedCmd;

#if RACE_BENCH_MODE
    /* Bench mode: steering still moves so it can be watched, wheels stay dead. */
    cmd->left    = 0.0f;
    cmd->right   = 0.0f;
    cmd->braking = false;
#endif
}
