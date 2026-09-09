#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "fsl_ctimer.h"
#include "fsl_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    CTIMER_Type   *pwmPeripheral;
    ctimer_match_t periodChannel;
    ctimer_match_t pwm1Channel;
    ctimer_match_t pwm2Channel;
    GPIO_Type     *motor1DirPort;
    uint32_t       motor1DirPin;
    GPIO_Type     *motor2DirPort;
    uint32_t       motor2DirPin;
} Hbridge;

extern Hbridge g_hbridge;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin);

/* Integer percent, -100..+100 per motor. Kept for the test programs. */
void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2);

/*
 * Same thing with a fractional command, writing the match registers directly.
 * CTIMER_UpdatePwmDutycycle only takes whole percent, and 1% steps are coarse
 * enough to be felt as a stutter when the speed planner is trimming the throttle
 * a fraction at a time through a corner.
 */
void HbridgeSpeedF(Hbridge *h, float speed1, float speed2);

void HbridgeBrake(Hbridge *h);

#ifdef __cplusplus
}
#endif

#endif
