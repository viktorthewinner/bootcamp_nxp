/*
 * driver.h - the thing that actually drives the car.
 *
 * Perception (track.c) says where the corridor is. Planning (racing_line.c) says
 * where to aim. This file decides what the servo and the two motors do about it:
 *
 *   steering  cross track error to the aim point, plus the heading right in front
 *             of the car, damped, with a soft deadband so small wiggles in the
 *             road are ignored instead of being chased
 *
 *   speed     the highest speed the current picture justifies, recomputed every
 *             frame. The strongest of four cues wins, and one of them looks at the
 *             far end of the image, so the car lifts and brakes before it arrives
 *             at a corner rather than in the middle of one
 *
 *   brakes    a short reverse pulse when a lot of speed has to go quickly
 *
 *   diff      the inside wheel is slowed in a corner, which rotates the car into
 *             the turn instead of pushing it wide
 *
 * No SDK calls live here, so the whole control chain can be run on a PC against a
 * simulated track.
 */
#ifndef DRIVER_H
#define DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "track.h"
#include "racing_line.h"

typedef struct
{
    float steer; /* -100..+100, positive is right. STEER_OFFSET not yet added. */
    float left;  /* motor 1 command, -100..+100 */
    float right; /* motor 2 command, -100..+100 */
    float speed; /* base speed before the differential, for telemetry */
    bool  braking;
} DriveCmd;

typedef struct
{
    TrackModel track;
    RacingLine line;
    float      severity;  /* 0 straight, 1 slowest corner  */
    float      steerTgt;
    uint16_t   lostFrames;
    uint32_t   frames;
    float      elapsedMs;
    bool       exiting;
} DriveState;

void Driver_Init(void);

/*
 * Call every loop.
 *   freshFrame - true when the camera returned a new picture this iteration
 *   segs / n   - the line segments from that picture (ignored if freshFrame is false)
 *   dt         - seconds since the previous call
 *
 * Planning only runs on a fresh frame. Rate limits, the brake pulse and the servo
 * slew run every call, so the outputs stay smooth even when the camera is slower
 * than the loop.
 */
void Driver_Step(bool freshFrame, const TrkSegment *segs, uint8_t n, float dt, DriveCmd *cmd);

const DriveState *Driver_State(void);

#endif /* DRIVER_H */
