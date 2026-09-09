#include "servo.h"
#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"

void Steer(float angle)
{
    float    duty;
    float    period;
    uint32_t pulse;

    if (angle > 100.0f)
    {
        angle = 100.0f;
    }
    if (angle < -100.0f)
    {
        angle = -100.0f;
    }

    /* -100..+100 maps onto 5%..10% of the 20 ms frame: the usual 1..2 ms servo pulse. */
    duty = 5.0f + ((angle + 100.0f) * (5.0f / 200.0f));

    /* CTIMER PWM matches on the way up, so the match value is where the pulse starts. */
    period = (float)CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];
    pulse  = (uint32_t)((period * (100.0f - duty)) / 100.0f);

    CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_0_CHANNEL] = pulse;
}

void TestServo(void)
{
    volatile int delay;
    int          strength;

    for (;;)
    {
        for (strength = -100; strength <= 100; strength++)
        {
            delay = 200000;
            while (delay != 0)
            {
                delay--;
            }
            PRINTF("Steer: %d\r\n", strength);
            Steer((float)strength);
        }
    }
}
