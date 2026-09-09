#ifndef ESC_H
#define ESC_H

#include "fsl_ctimer.h"
#include <stdint.h>

typedef struct {
    CTIMER_Type *pwmPeripheral;
    ctimer_match_t periodChannel;
    ctimer_match_t pwmChannel;
    double minDuty;
    double maxDuty;
} Esc;

void EscInit(Esc *e, CTIMER_Type *periph, ctimer_match_t periodCh, ctimer_match_t pwmCh);
void EscSetSpeed(Esc *e, double speed);
void EscBrake(Esc *e);

#endif
