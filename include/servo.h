#ifndef STEER_H_
#define STEER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Steering angle, -100 (full left) to +100 (full right), mapped onto the standard
 * 1..2 ms servo pulse inside the 20 ms frame CTIMER2 generates.
 *
 * This is the raw servo command: STEER_OFFSET has to be added by the caller, since
 * the wheels only point straight at STEER_OFFSET rather than at 0.
 *
 * float, not double. The FPU on this part is single precision only, so a double
 * here would be emulated in software on every control cycle.
 */
void Steer(float angle);

/* Sweeps the servo end to end forever. Handy for finding STEER_OFFSET and the
 * mechanical limits. Never returns. */
void TestServo(void);

#ifdef __cplusplus
}
#endif

#endif
