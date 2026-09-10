/*
 * intersection.h - spotting a crossing by the hole it leaves, and driving over it.
 *
 * An intersection is the one feature on the circuit where the track model is
 * actively wrong. Two black lines arrive, two more cut across them, and track.c
 * throws the crossing ones away as start lines - so the corridor it reports opens
 * out sideways for a few frames and the racing line happily aims at the gap. The
 * car turns down the crossing track. The rules say go straight on.
 *
 * WHAT A CROSSING LOOKS LIKE
 *
 * Not a corner. A hole. The edge the car has been following stops, there is white
 * space the width of the crossing track, and then the same edge starts again on
 * the far side, parallel to where it left off and in line with it:
 *
 *        |                    |
 *        |  far edge          |          <- picks up again, parallel and in line
 *        |                    |
 *
 *         (white space, about one track width)
 *
 *        |                    |
 *        |  near edge         |          <- the one the car is following
 *        |                    |
 *                 car
 *
 * That is the whole signature, and it is worth more than any amount of corner
 * geometry because it survives the camera reporting the edges as a ragged chain of
 * short vectors, which is what it actually does. The bars across the mouth of the
 * crossing are not used at all - they are thrown away with everything else that
 * does not run up the track.
 *
 * A plain corner does not look like this. Its edges are continuous; when one leaves
 * the side of the picture nothing appears beyond it. A start line does not either -
 * it is a bar across an unbroken pair of edges. Requiring the far piece to exist,
 * to be parallel, and to be in line with the near one is what separates a crossing
 * from an edge that has simply run out of frame.
 *
 * MEASURED ON THE GROUND, NOT IN THE PICTURE
 *
 * "A gap about one track width long" is a statement about the track, and in the
 * picture the same gap is thirty rows deep at the bumper and three near the
 * horizon. So every endpoint is unprojected onto the ground first - flat surface,
 * pinhole camera, two divisions - and the scan for the hole is done in
 * centimetres. That costs one calibration, CAM_HORIZON_ROW and CAM_HEIGHT_CM in
 * race_config.h, and nothing else in the firmware needs them. Get them wrong and
 * the car misses crossings rather than inventing them.
 *
 * WHAT THE CAR DOES ABOUT IT
 *
 * Lines itself up parallel to the track and drives. Parallel, not centred: on the
 * ground the visible edges have a heading relative to the car, the steering nulls
 * that heading, and holding it means the car comes out of the crossing on the same
 * line it went in on. Aiming at a computed centre would need the corridor width,
 * which is exactly the thing that is not measurable inside a crossing.
 *
 * How far to drive is not a guess either - the gap was measured on the way in, so
 * the latch runs for that distance plus the length of the car.
 *
 * Pure arithmetic, no SDK, no hardware, no file scope state - the caller owns the
 * struct - so it can be compiled and exercised on a PC like the rest of the chain.
 * test/track_sim.c -isec does exactly that.
 */
#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <stdbool.h>
#include <stdint.h>
#include "track.h"

typedef enum
{
    ISEC_IDLE = 0,  /* nothing confirmed, or waiting for the gap to come closer  */
    ISEC_CROSSING,  /* committed: the module owns the steering until the budget runs out */
    ISEC_COOLDOWN   /* just crossed one, ignore detections so it cannot re-trigger */
} IsecPhase;

typedef struct
{
    /* ---- what the latest frame saw ---- */
    bool  seen;       /* at least one side showed a crossing shaped hole */
    bool  sawLeft;    /* the hole was found in the left hand edge         */
    bool  sawRight;
    float gapStartCm; /* how far ahead the edges stop                     */
    float gapEndCm;   /* how far ahead they start again                   */
    float gapCm;      /* the white space between, about one track width   */
    float slope;      /* heading of the visible edges on the ground, sideways per
                       * forward. 0 = the car is parallel to the track,
                       * positive = the track runs off to the left ahead  */
    uint8_t nEdges;   /* vectors that ran up the track this frame, for diagnostics */

    /* ---- what the car should do ---- */
    float steer;      /* steering command, same units as DriveCmd.steer   */
    float authority;  /* 0 = driver ignores this module, 1 = it owns the steering */

    /* ---- the latch ---- */
    IsecPhase phase;
    bool      crossing; /* phase == ISEC_CROSSING, the flag the driver reads */
    float     budgetM;  /* metres of the current phase still to run          */
    float     phaseMs;  /* time in the current phase, the backstop for a stopped car */
    uint8_t   hits;     /* confirmation count, up on a detection and down on a miss */
    uint16_t  count;    /* crossings committed to since boot, for telemetry  */
} Intersection;

/* Clears the whole struct back to idle. Call once, from Driver_Init. */
void Intersection_Init(Intersection *st);

/*
 * Call once per camera frame, with the RAW vectors - the same array handed to
 * Track_Update, before it drops anything. Track_Update throws away everything
 * close to horizontal and keeps only what fits its corridor; this needs the
 * unfiltered list so it can see for itself which vectors run up the track and
 * where they stop.
 */
void Intersection_Update(const TrkSegment *segs, uint8_t n, Intersection *st);

/*
 * Call every control loop with the ground covered and the time elapsed since the
 * previous call. This is what ends a crossing: the latch is a distance, not a
 * frame count, because a frame is not a fixed amount of track. dt is the backstop
 * for the case distance cannot end - a car that has stopped moving.
 */
void Intersection_Advance(Intersection *st, float distM, float dt);

#endif /* INTERSECTION_H */
