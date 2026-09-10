/*
 * recover.h - getting the second black line back into the picture.
 *
 * The Pixy2 sees about 60 degrees across, which is narrower than the track is
 * wide close up, so there are ordinary pieces of road where only one of the two
 * black lines reaches the camera. track.c copes by inferring the missing edge
 * from its learned width model, and for a frame or two that is exactly right.
 *
 * What it cannot do is notice that the guess has stopped being checked. With one
 * edge measured the car has no observability of where it sits across the track:
 * the line it can see stays visible however far the car drifts away from the one
 * it cannot, so the error never corrects itself. The car is dead reckoning on a
 * width model - and it does it hardest in a corner, which is where it can least
 * afford to, because the corner's real radius is exactly what the missing edge
 * would have told it.
 *
 *      +---------------------------+
 *      |    ____________           |     the left line is measured
 *      |   /                       |
 *      |  |                        |     the right one left the frame
 *      |  |                        |     sideways, and everything the car
 *      |  |                        |     believes about it is a guess
 *      +---------------------------+
 *
 * The fix is the one a driver uses. If you cannot see the far side of the road,
 * stop leaning on the line you have: straighten up, then ease very slightly
 * toward the side you have lost. That yaws the camera far enough for the missing
 * line to come back into frame, and the moment it does the corridor is measured
 * again and the corner can be driven on real numbers instead of on the width
 * model.
 *
 * Two things stop that from being a way to drive off the track.
 *
 *   - It only arms while the road right in front of the bumper is straight. A
 *     genuine hairpin loses its outside line too, and there the answer is to
 *     turn, not to straighten. The near heading is the one cue a narrow view
 *     cannot take away, which is the same reason intersection.c leans on it.
 *
 *   - It will straighten but not lean when the line it has lost is the OUTSIDE
 *     of a bend it can already see. Easing toward the outside of a corner is
 *     how a car leaves one, and it would not even work: the outside line is
 *     out of frame for a geometric reason that holds for the whole corner, so
 *     there is nothing there to find. Giving up the racing line still helps,
 *     because that racing line was computed from one edge and a guess.
 *
 *   - It gives up. If the missing line has not come back within RCV_MAX_FRAMES
 *     then the road really is only one line wide from here, so the probe stands
 *     down and does not try again until both edges have been seen together.
 *     One probe per blind stretch, never a limit cycle inside a corner.
 *
 * The probe is expressed as a shift of the aim point rather than as a steering
 * angle, so it passes through the hard safety clamp in racing_line.c like
 * everything else and can never put the car's path over an edge the model knows
 * about.
 *
 * Pure arithmetic, no SDK, so it runs in the host simulator with the rest of the
 * control chain.
 */
#ifndef RECOVER_H
#define RECOVER_H

#include <stdint.h>
#include <stdbool.h>
#include "track.h"

typedef enum
{
    RCV_IDLE = 0, /* both lines in sight, or not blind for long enough  */
    RCV_PROBE,    /* straightening, then easing toward the missing side */
    RCV_BACKOFF   /* gave up on this stretch, not trying again yet      */
} RcvPhase;

typedef struct
{
    RcvPhase phase;
    bool     active; /* the aim point should be shifted this frame        */
    float    side;   /* +1 the missing line is to the right, -1 to left   */
    float    nudge;  /* 0..1, how far into the probe - 0 is straighten    */
    float    lean;   /* -1..+1 aim point shift; 0 when only straightening */
    uint16_t blind;  /* consecutive frames with only one edge measured    */
    uint16_t held;   /* frames spent in the current phase                 */
    uint16_t probes; /* probes started, for telemetry                     */
    uint16_t found;  /* probes that got the missing line back             */
} RecoverState;

void Recover_Init(void);

/*
 * Call once per camera frame, before the racing line is planned, whether or not
 * the track model came back usable.
 *   m    - this frame's corridor
 *   busy - true when something else already owns the steering (a crossing), in
 *          which case the probe stands down rather than fighting it
 *   pinched - last frame's racing line had to be pulled back by the safety
 *          check. That means the corridor has no room spare, and a corridor
 *          with no room spare is not one to go leaning about in - whatever
 *          the probe might have learned is not worth the margin it costs.
 */
void Recover_Update(const TrackModel *m, bool busy, bool pinched,
                    RecoverState *out);

#endif /* RECOVER_H */
