#include "esc.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"

void EscInit(Esc *e, CTIMER_Type *periph, ctimer_match_t periodCh, ctimer_match_t pwmCh)
{
    e->pwmPeripheral = periph;
    e->periodChannel = periodCh;
    e->pwmChannel = pwmCh;
    e->minDuty = 5.0;
    e->maxDuty = 10.0;
}

void EscSetSpeed(Esc *e, double speed)
{
    if(speed < 0){
    	speed = 0;
    }
    else if(speed > 100){
    	speed = 100;
    }
    double duty = e->minDuty + (speed / 100.0) * (e->maxDuty - e->minDuty);
    uint32_t periodTicks = e->pwmPeripheral->MR[e->periodChannel];
    uint32_t pulseTicks = (uint32_t)((periodTicks * (100.0 - duty)) / 100.0);
    e->pwmPeripheral->MR[e->pwmChannel] = pulseTicks;
}

void EscBrake(Esc *e)
{
    EscSetSpeed(e, 0.0);
}
