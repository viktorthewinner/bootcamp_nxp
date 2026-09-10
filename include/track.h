/*
 * track.h - turns Pixy2 line vectors into a driveable corridor.
 *
 * The camera does not hand over "the track". It hands over a handful of line
 * segments, in no particular order, some of which are the left edge, some the right
 * edge, and some of which are the start line or plain noise. This module sorts that
 * out and answers three questions the driver actually needs:
 *
 *    where are the two black lines, at eight distances ahead of the car
 *    how far ahead can the car still see
 *    which way is the track pointing, close up and far away
 *
 * Pure arithmetic, no SDK, no hardware. That is deliberate: it means the whole
 * perception and planning chain can be compiled and exercised on a PC.
 */
#ifndef TRACK_H
#define TRACK_H

#include <stdint.h>
#include <stdbool.h>
#include "race_config.h"

/* One line segment. y0 >= y1 always: the tail is the end nearest the car. */
typedef struct
{
    float x0, y0;
    float x1, y1;
} TrkSegment;

typedef struct
{
    /* Per sample row. Index 0 is nearest the bumper, TRK_ROWS-1 is farthest ahead. */
    float y[TRK_ROWS];      /* image row this sample came from            */
    float xl[TRK_ROWS];     /* left black line, image column              */
    float xr[TRK_ROWS];     /* right black line, image column             */
    float center[TRK_ROWS]; /* middle of the corridor                     */
    float width[TRK_ROWS];  /* corridor width in pixels                   */
    float margin[TRK_ROWS]; /* keep-out band beside each line             */
    bool  sawL[TRK_ROWS];   /* left edge really seen, not inferred        */
    bool  sawR[TRK_ROWS];
    bool  valid[TRK_ROWS];

    uint8_t segCount;  /* usable vectors this frame, after filtering      */
    uint8_t nValid;    /* rows 0..nValid-1 are usable and contiguous      */
    uint8_t topRow;    /* highest usable row index                        */
    bool    haveTrack; /* enough of a corridor to drive on                */
    bool    bothEdges; /* both black lines were genuinely seen somewhere  */

    /* Heading, in pixels of sideways travel per row of distance ahead.
     * Positive means the track bends to the right. */
    float headNear; /* the road right in front of the bumper */
    float headFar;  /* the road at the limit of vision       */
    float curv;     /* headFar - headNear, the rate the corner is tightening */
} TrackModel;

/* Resets the learned corridor width back to the seeds in race_config.h. */
void Track_Init(void);

/* Rebuilds the corridor from one camera frame.
 * segs/n may be empty, in which case the model comes back with haveTrack false.
 * Returns true when the result is good enough to drive on. */
bool Track_Update(const TrkSegment *segs, uint8_t n, TrackModel *out);

/* Corridor width the model currently believes in, per row. For diagnostics. */
float Track_LearnedWidth(uint8_t row);

/* The same width profile evaluated at any image row, not just the eight sample
 * ones. intersection.c needs it: a crossing corner turns up wherever it turns up,
 * and half this width is how far from it the middle of the gap is. */
float Track_WidthAtY(float y);

/* The camera mounting, worked out from the width the car has measured rather
 * than from a ruler: the width model reaches zero at the horizon, and its slope
 * is the real track width divided by the camera height. intersection.c uses
 * these instead of the constants in race_config.h once the model has settled. */
float Track_HorizonRow(void);
float Track_CamHeightCm(void);

#endif /* TRACK_H */
