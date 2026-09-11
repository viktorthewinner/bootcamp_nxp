#include "speed_ctl.h"
#include <math.h>
#include "race_config.h"

static SpeedState s;
static float      s_vRefPrev;
static float      s_vBrakeFrom; /* speed the current braking event started at */
static float      s_vBrakeTo;   /* speed it is heading for                    */

/*
 * Steady-state speed in m/s at duty 0, 10, 20 ... 100 percent, from
 * sim/plant.py with the car's own rolling resistance included.
 *
 * Two tables because the bridge does two different things. Driving one input
 * from a GPIO and PWMing the other gives coast-during-off in one direction and
 * brake-during-off in the other, and they are not the same machine: brake decay
 * is very nearly a straight line through the origin, coast decay has a 30%
 * deadband and an S-shaped middle. The car has coast decay going forwards
 * today, which is the worse of the two - see the note on MOTOR_FAST_DECAY_FWD
 * in race_config.h.
 */
#if MOTOR_FAST_DECAY_FWD
static const float k_vAtDuty[11] = {
    0.0000f, 0.0000f, 0.0000f, 0.2071f, 0.7425f, 1.1009f,
    1.3480f, 1.5239f, 1.6530f, 1.7504f, 2.0121f
};
#else
static const float k_vAtDuty[11] = {
    0.0000f, 0.0087f, 0.2313f, 0.4539f, 0.6765f, 0.8991f,
    1.1217f, 1.3443f, 1.5669f, 1.7895f, 2.0121f
};
#endif

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

float SpeedCtl_SpeedForDuty(float dutyPercent)
{
    float d = clampf(dutyPercent, 0.0f, 100.0f) * 0.1f; /* index space, 0..10 */
    int   i = (int)d;
    float f;

    if (i >= 10)
    {
        return k_vAtDuty[10];
    }
    f = d - (float)i;
    return k_vAtDuty[i] + ((k_vAtDuty[i + 1] - k_vAtDuty[i]) * f);
}

float SpeedCtl_DutyForSpeed(float vMs)
{
    int i;

    if (vMs <= 0.0f)
    {
        return 0.0f;
    }
    if (vMs >= k_vAtDuty[10])
    {
        return 100.0f;
    }

    /* The table is monotonic, so a walk is enough and costs nothing at 11
     * entries. Flat leading entries (the coast-decay deadband) are skipped by
     * requiring a real rise before interpolating, otherwise a tiny request
     * would divide by zero. */
    for (i = 0; i < 10; i++)
    {
        if ((vMs <= k_vAtDuty[i + 1]) && (k_vAtDuty[i + 1] > k_vAtDuty[i]))
        {
            float f = (vMs - k_vAtDuty[i]) / (k_vAtDuty[i + 1] - k_vAtDuty[i]);
            if (f < 0.0f)
            {
                f = 0.0f;
            }
            return ((float)i + f) * 10.0f;
        }
    }
    return 100.0f;
}

void SpeedCtl_Init(void)
{
    s.vTargetMs = 0.0f;
    s.vRefMs    = 0.0f;
    s.vEstMs    = 0.0f;
    s.duty      = 0.0f;
    s.integ     = 0.0f;
    s.measured  = false;
    s.braking   = false;
    s.brakeProg = 1.0f;
    s_vRefPrev  = 0.0f;
    s_vBrakeFrom = 0.0f;
    s_vBrakeTo   = 0.0f;
}

const SpeedState *SpeedCtl_State(void)
{
    return &s;
}

void SpeedCtl_Measure(float vMs)
{
    s.vEstMs   = vMs;
    s.measured = true;
}

float SpeedCtl_SpeedFrac(void)
{
    /*
     * Normalised against the fastest the PLANNER will ever ask for, not against
     * what the car could do flat out. The look-ahead row and the racing line
     * both slide on this, and they were tuned to reach the end of their travel
     * at SPEED_MAX. Dividing by SPEED_TOP_MS instead caps the fraction at
     * SPEED_MAX/100, so the look-ahead never extends fully, the effective
     * steering gain stays high, the car steers more, the severity cue reads that
     * as corner, and it slows down - a tidy little loop that costs a second a lap
     * for no reason at all.
     */
    float vFull = (0.01f * SPEED_MAX) * SPEED_TOP_MS;

    if (vFull < 0.05f)
    {
        vFull = 0.05f;
    }
    return clampf(s.vEstMs / vFull, 0.0f, 1.0f);
}

float SpeedCtl_Step(float vTargetUnits, float dt, bool allowBrake, float accelMs2)
{
    float vTarget, dv, dvdt, duty, accLim, decLim;
    float decMs2, brakeCap;

    dt = clampf(dt, 1.0e-4f, 0.2f);

    /*
     * How hard may the car brake right now. A power curve of how fast it is
     * going: gentle when slow, the full values at 100%, and with the exponent
     * at 2 it tracks kinetic energy - the thing the brake actually has to
     * get rid of.
     */
    {
        float hard = clampf((SpeedCtl_SpeedFrac() - SPEED_BRAKE_RAMP_FROM) /
                                (1.0f - SPEED_BRAKE_RAMP_FROM),
                            0.0f, 1.0f);

        hard = powf(hard, SPEED_BRAKE_EXP);

        decMs2   = SPEED_DEC_MS2 + ((SPEED_DEC_FULL_MS2 - SPEED_DEC_MS2) * hard);
        brakeCap = SPEED_BRAKE_DUTY_MAX + ((SPEED_BRAKE_DUTY_FULL - SPEED_BRAKE_DUTY_MAX) * hard);
    }

    vTarget     = clampf(vTargetUnits, 0.0f, 100.0f) * (0.01f * SPEED_TOP_MS);
    s.vTargetMs = vTarget;

    /*
     * Shape the request to something the car could actually follow. This is not
     * politeness: the feedforward below differentiates the reference, so an
     * unshaped step would ask for an unbounded duty and then hand the bridge a
     * spike that means nothing. Bounding it here is what makes the derivative
     * a real acceleration request.
     */
    accLim = ((accelMs2 > 0.0f) ? accelMs2 : SPEED_ACC_MS2) * dt;
    decLim = (allowBrake ? decMs2 : SPEED_COAST_MS2) * dt;

    dv = vTarget - s.vRefMs;
    if (dv > accLim)
    {
        dv = accLim;
    }
    else if (dv < -decLim)
    {
        dv = -decLim;
    }
    else
    {
        /* the request is already reachable this step */
    }
    s.vRefMs += dv;
    if (s.vRefMs < 0.0f)
    {
        s.vRefMs = 0.0f;
    }

    dvdt       = (s.vRefMs - s_vRefPrev) / dt;
    s_vRefPrev = s.vRefMs;

    /*
     * Where are we in the braking event. A braking event starts when the
     * reference has to come down to meet a lower target and ends when it gets
     * there (or the target climbs back above it). Progress runs 0 -> 1 over
     * the speed still to be shed, which is what the steering uses to feed the
     * lock in gradually and have all of it by the moment braking ends.
     */
    if (allowBrake && (vTarget < (s.vRefMs - 0.02f)))
    {
        if (!s.braking)
        {
            s.braking    = true;
            s_vBrakeFrom = s.vRefMs;
            s_vBrakeTo   = vTarget;
        }
        else if (vTarget < s_vBrakeTo)
        {
            s_vBrakeTo = vTarget; /* target dropped further mid-brake: longer event */
        }
        else
        {
            /* same event, same destination */
        }

        {
            float span = s_vBrakeFrom - s_vBrakeTo;

            s.brakeProg = (span > 0.02f) ? clampf((s_vBrakeFrom - s.vRefMs) / span, 0.0f, 1.0f) : 1.0f;
        }
    }
    else
    {
        s.braking   = false;
        s.brakeProg = 1.0f;
    }

    /*
     * Feedforward. The static part inverts the measured duty->speed map, so the
     * request comes out as the duty that really produces it. The dynamic part
     * inverts the 348 ms pole: to make speed follow a ramp, the winding needs
     * tau * dv/dt more voltage than the steady value, and K converts that back
     * into duty. Together they are an open-loop inverse of the plant, which is
     * why the car changes speed when it is asked to instead of afterwards.
     */
    duty = SpeedCtl_DutyForSpeed(s.vRefMs);
    duty += (SPEED_FF_LAG * MOTOR_TAU_S * dvdt / MOTOR_K_MS_PER_DUTY) * 100.0f;

#if SPEED_HAVE_FEEDBACK
    /*
     * With a real speed measurement, add the integral term. It is the only
     * thing that rejects a drag disturbance, because a disturbance is by
     * definition what the model does not know about.
     */
    {
        float e   = s.vRefMs - s.vEstMs;
        float un  = duty + (SPEED_KP * e) + (SPEED_KI * s.integ);
        float sat = clampf(un, allowBrake ? -brakeCap : 0.0f, 100.0f);

        /* Conditional integration. Back-calculating instead throws the
         * accumulated value away every time the loop leaves a stop, and the car
         * then has to earn it back at 1/Ki - which is a slow crawl out of every
         * corner exit, exactly where it hurts. */
        if ((un == sat) || ((e * sat) < 0.0f))
        {
            s.integ += e * dt;
            s.integ = clampf(s.integ, -SPEED_I_LIMIT, SPEED_I_LIMIT);
        }
        duty = sat;
    }
#endif

    duty = clampf(duty, allowBrake ? -brakeCap : 0.0f, 100.0f);
    s.duty = duty;

    /*
     * Keep the model estimate moving even when a sensor is present, so the two
     * can be compared in telemetry. Without a sensor this IS the speed the rest
     * of the firmware reasons about: it is dead reckoning, so it inherits the
     * model's gain error, but it has the right dynamics - and the dynamics are
     * what the look-ahead and the racing line actually care about.
     */
    if (!s.measured)
    {
        float vSs = SpeedCtl_SpeedForDuty(duty);
        s.vEstMs += ((vSs - s.vEstMs) / MOTOR_TAU_S) * dt;
        if (s.vEstMs < 0.0f)
        {
            s.vEstMs = 0.0f;
        }
    }
    s.measured = false;

    return duty;
}
