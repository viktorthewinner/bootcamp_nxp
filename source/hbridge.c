#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"

Hbridge g_hbridge;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin)
{
    h->pwmPeripheral = pwmPeriph;
    h->periodChannel = periodCh;
    h->pwm1Channel   = pwm1Ch;
    h->pwm2Channel   = pwm2Ch;
    h->motor1DirPort = m1DirPort;
    h->motor1DirPin  = m1DirPin;
    h->motor2DirPort = m2DirPort;
    h->motor2DirPin  = m2DirPin;
}

void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2)
{
    HbridgeSpeedF(h, (float)speed1, (float)speed2);
}

static void hbridge_write_duty(Hbridge *h, ctimer_match_t channel, float duty)
{
    uint32_t period = h->pwmPeripheral->MR[h->periodChannel];
    uint32_t pulse;

    if (duty <= 0.0f)
    {
        /* Push the match past the period so it never fires: a true 0%. */
        pulse = period + 1u;
    }
    else if (duty >= 100.0f)
    {
        pulse = 0u;
    }
    else
    {
        pulse = (uint32_t)(((float)period * (100.0f - duty)) / 100.0f);
    }

    h->pwmPeripheral->MR[channel] = pulse;
}

void HbridgeSpeedF(Hbridge *h, float speed1, float speed2)
{
    float duty1, duty2;

    if (speed1 > 100.0f)
    {
        speed1 = 100.0f;
    }
    if (speed1 < -100.0f)
    {
        speed1 = -100.0f;
    }
    if (speed2 > 100.0f)
    {
        speed2 = 100.0f;
    }
    if (speed2 < -100.0f)
    {
        speed2 = -100.0f;
    }

    /* Reverse is direction pin high plus an inverted duty, which is how this
     * bridge is wired: at DIR = 1 the driver sees 100 - duty. */
    duty1 = (speed1 >= 0.0f) ? speed1 : (100.0f + speed1);
    duty2 = (speed2 >= 0.0f) ? speed2 : (100.0f + speed2);

    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, (speed1 < 0.0f) ? 1U : 0U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, (speed2 < 0.0f) ? 1U : 0U);

    hbridge_write_duty(h, h->pwm1Channel, duty1);
    hbridge_write_duty(h, h->pwm2Channel, duty2);
}

void HbridgeBrake(Hbridge *h)
{
    /* Both inputs driven high shorts the motor windings: a real brake, not a coast. */
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);

    hbridge_write_duty(h, h->pwm1Channel, 100.0f);
    hbridge_write_duty(h, h->pwm2Channel, 100.0f);
}
