/*
 * ticks.h - free running microsecond clock built on the Cortex-M33 SysTick.
 *
 * SysTick is unused by the SDK on this board, so it is borrowed here as a plain
 * 24 bit down counter with no interrupt. The control loop needs a real dt: without
 * one the D term and every rate limit become meaningless whenever the camera
 * returns a frame late.
 *
 * Ticks_Us() must be called at least once every ~110 ms (24 bits at 150 MHz),
 * otherwise a counter wrap is missed and time stands still. The control loop runs
 * every few milliseconds, so this is never close.
 */
#ifndef TICKS_H
#define TICKS_H

#include <stdint.h>

void     Ticks_Init(uint32_t coreClockHz);

/* Microseconds since Ticks_Init(). Wraps after ~71 minutes. */
uint32_t Ticks_Us(void);

/* Milliseconds since Ticks_Init(). */
uint32_t Ticks_Ms(void);

#endif /* TICKS_H */
