/*
 * racing_line.h - picks the point the car should aim at.
 *
 * Driving down the middle of the corridor is the slow way round. A racing driver
 * uses the whole width: wide on the way in, tight at the apex, wide again on the
 * way out. That straightens the corner, which means a bigger radius, which means a
 * higher speed through it.
 *
 * The camera cannot tell the car where it is in a corner, but it can be worked out
 * from two headings. headNear is where the track points right in front of the
 * bumper, headFar is where it points at the limit of vision:
 *
 *    straight ahead, corner in the distance -> turn in has not happened yet -> ENTRY, stay wide
 *    turning here and still turning ahead   -> the middle of the corner     -> APEX, hug the inside
 *    turning here but straight ahead        -> the corner is opening up     -> EXIT, run wide again
 *
 * Those three blend continuously rather than switching, so the aim point sweeps
 * across the track instead of jumping.
 *
 * Whatever the racing line asks for, the result is then forced back inside the
 * black lines. That check is the last thing this module does, and it is not
 * optional: a fast lap that leaves the track is a lap that does not count.
 */
#ifndef RACING_LINE_H
#define RACING_LINE_H

#include <stdbool.h>
#include <stdint.h>
#include "track.h"
#include "recover.h"

typedef struct
{
    float   targetX;  /* image column to aim at                                  */
    uint8_t laRow;    /* row the aim point sits on                               */
    float   bias;     /* -1 hard against the left margin .. +1 against the right */
    bool    chicane;  /* a small chicane is being deliberately ignored           */
    bool    clamped;  /* the safety check had to pull the aim point back         */
    float   wEntry;   /* phase weights, exposed for tuning and debug             */
    float   wApex;
    float   wExit;
    float   probe;    /* one-sided recovery lean, -1..+1, 0 when idle  */
    float   conf;     /* how well this frame described the road, LINE_CONF_MIN..1 */
    bool    straight; /* the road is straight, so two vectors describe it fully   */
    bool    chorded;  /* a bend, described by too few vectors to measure a bend   */
} RacingLine;

void RL_Init(void);

/* speedFrac is 0 at a standstill and 1 at SPEED_MAX; it sets how far ahead the car
 * looks. Slow means look close and be accurate, fast means look far and be smooth. */
/* rcv is the one-sided recovery state from recover.c; see recover.h. It may
 * shift the aim point toward a black line the camera has lost, and that shift
 * goes in before the safety check below, never after it. */
void RL_Compute(const TrackModel *m, float speedFrac, const RecoverState *rcv,
                RacingLine *out);

#endif /* RACING_LINE_H */
