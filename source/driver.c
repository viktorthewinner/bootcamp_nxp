#include "driver.h"
#include "race_config.h"
#include "speed_ctl.h"
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

    s_steer      = 0.0f;
    s_steerTgt   = 0.0f;
    s_errF       = 0.0f;
    s_errFPrev   = 0.0f;
    s_vTarget    = 0.0f;
    s_speedCmd   = 0.0f;
    s_speedFrac  = 0.0f;
    s_sinceFrame  = 0.0f;
    s_blindMs     = 0.0f;
    s_blindM      = 0.0f;
    s_frameDt     = 0.020f;
    s_frameDtBest = 0.020f;
    s_brakeMs     = 0.0f;
    s_brakeMag    = 0.0f;

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

    /* Never drive faster than the distance that can actually be seen. */
    v *= see_factor(m->nValid);

    /* One edge visible means the other one is a guess. Guesses get less speed. */
    if (!m->bothEdges)
    {
        v *= SPEED_ONE_EDGE_CAP;
    }

    /* The safety check had to pull the aim point back, so the corridor is tighter
     * than the racing line wanted. Take a little more out. */
    if (s_st.line.clamped)
    {
        v *= 0.90f;
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

        if (Track_Update(segs, n, &s_st.track))
        {
            float dtFrame = clampf(s_sinceFrame, 1.0e-4f, 0.2f);

            s_st.lostFrames = 0u;
            RL_Compute(&s_st.track, s_speedFrac, &s_st.line);
            plan_steering(dtFrame);
            plan_speed();
        }
        else if (s_st.lostFrames < 0xFFFFu)
        {
            s_st.lostFrames++;
        }
        else
        {
            /* counter pinned, nothing more to do */
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
    if (s_st.lostFrames > 0u)
    {
#if SPEED_CLOSED_LOOP
        s_blindM += SpeedCtl_State()->vEstMs * dt;
#else
        /* The open-loop path never runs the observer, so read the speed straight
         * off the static map instead. It ignores the motor lag and so runs a
         * little ahead of the truth, which errs toward stopping sooner. */
        s_blindM += SpeedCtl_SpeedForDuty(s_speedCmd) * dt;
#endif
    }
    else
    {
        s_blindM = 0.0f;
    }

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
