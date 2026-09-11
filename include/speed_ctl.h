#ifndef SPEED_CTL_H
#define SPEED_CTL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * speed_ctl - turns the planner's speed request into a bridge duty.
 *
 * Without this the firmware sends the planner's number straight to the H-bridge
 * as a PWM duty, which means "speed 75" is not a speed at all. It is 75% of the
 * battery voltage, and what that is worth depends on the battery, on which way
 * the bridge decays, and on how hard the car is being dragged at the time. The
 * identification in sim/plant.py measured all three:
 *
 *   - duty -> speed is a first-order lag, K = 2.23 (m/s)/duty, tau = 348 ms
 *   - the map has a 10% offset (slow decay) or a 30% deadband (fast decay)
 *   - a 1 N drag costs 18-36% of the speed, and nothing notices
 *
 * So this module does three things:
 *
 *   1. shapes the request into something the car can actually follow, at a
 *      bounded acceleration, so the derivative below is meaningful
 *   2. inverts the static map, so a request in m/s comes out as the duty that
 *      really produces it
 *   3. inverts the 348 ms pole by feeding forward tau * dv/dt, which is what
 *      makes the car change speed at the rate it was asked to rather than
 *      three tenths of a second later
 *
 * Steps 2 and 3 need no sensor - they are arithmetic on a model. Step 4, the
 * integral term that rejects the drag disturbance, does need one: define
 * SPEED_HAVE_FEEDBACK and call SpeedCtl_Measure() and it turns itself on.
 */

typedef struct
{
    float vTargetMs; /* what the planner asked for, m/s                     */
    float vRefMs;    /* after rate shaping - what is actually being chased   */
    float vEstMs;    /* best estimate of the real speed, m/s                 */
    float duty;      /* what went to the bridge, percent                     */
    float integ;     /* integral state, only used with feedback              */
    bool  measured;  /* true when vEstMs came from a sensor, not the model   */
    bool  braking;   /* a braking event is in progress                      */
    float brakeProg; /* 0 = braking just started, 1 = finished or not braking */
} SpeedState;

void SpeedCtl_Init(void);

/*
 * One control step.
 *
 *   vTargetUnits  0..100 from the speed planner, where 100 means SPEED_TOP_MS
 *   dt            seconds since the last call
 *   allowBrake    false while starting up, so the controller never commands
 *                 reverse when the car is not yet under control
 *   accelMs2      ceiling on how fast the reference may rise. Raise it on a
 *                 corner exit: the reference then runs ahead of the car, the
 *                 feedforward pins the duty at 100%, and that is what "get on
 *                 the power" means in a machine that has no throttle pedal.
 *
 * Returns the base duty in percent, -100..+100, before torque vectoring.
 */
float SpeedCtl_Step(float vTargetUnits, float dt, bool allowBrake, float accelMs2);

/* Duty in percent that holds a given speed in m/s, from the identified map. */
float SpeedCtl_DutyForSpeed(float vMs);

/* Steady-state speed in m/s for a given duty percent - the same map forwards. */
float SpeedCtl_SpeedForDuty(float dutyPercent);

/* Best estimate of the real speed, as a 0..1 fraction of SPEED_TOP_MS. This is
 * what the look-ahead and the racing line should slide on, not the command. */
float SpeedCtl_SpeedFrac(void);

const SpeedState *SpeedCtl_State(void);

/*
 * Feed a measured wheel speed in m/s. Call it whenever a sensor produces one.
 * With SPEED_HAVE_FEEDBACK set this closes the loop and the integral term
 * starts rejecting drag; without it the value is recorded for telemetry and
 * otherwise ignored.
 */
void SpeedCtl_Measure(float vMs);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_CTL_H */
