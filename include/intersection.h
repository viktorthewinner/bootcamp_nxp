/*
 * intersection.h - recognising a crossing, and taking it flat out.
 *
 * The camera never says "intersection". It hands over the same unordered pile of
 * line vectors it always does. What makes a crossing recognisable is the shape of
 * that pile, and the shape comes straight from how the track is painted:
 *
 *     ====        ====        the crossing track's own two edges, cut in half
 *        |        |           by our track passing over them
 *        |        |
 *     ====        ====
 *
 *        ^        ^           our two lines, interrupted for exactly the width
 *        |        |           of the crossing track, then resuming in line
 *
 * So the tell is a pair of nearly sideways vectors lying one either side of the
 * corridor, at the same distance ahead, adding up to more than a track width -
 * with the road through them straight. track.c already finds those sideways
 * vectors and discards them, because steering off one would end a race. This
 * module reads the same list and asks what their arrangement means.
 *
 * That is one of three shapes, and which one arrives is the camera's decision
 * rather than the track's - it depends on how the tracker cuts the paint where
 * our line runs into the crossing's:
 *
 *     ====        ====        it breaks the paint and drops the joint: loose
 *        |        |           bars, with a track-width hole between them
 *
 *      \             /        it follows the corner round: our own vectors
 *       |           |         bend outward and splay apart, which perspective
 *       |           |         says two parallel edges can never do
 *
 *     ----o        o----      it breaks the paint but keeps both pieces: a
 *         |        |          right-angle elbow either side, arms pointing
 *         |        |          away from each other
 *
 * All three are one junction, and the car acts on any of them. The last is the
 * one a Pixy2 produced on the real track, and neither of the others sees it:
 * the lines still converge normally so nothing splays, and a crossing edge near
 * the top of the frame is drawn across a few rows rather than one, so the
 * flatness test throws out an arm and leaves a single bar where two are wanted.
 *
 * None of the three rests on looking like a crossing. Each rests on something a
 * single piece of road cannot do - leave a hole in itself, lean both edges
 * outward at once, or turn left and right in the same frame.
 *
 *
 * The reason it is worth doing is not the recognising, it is what the car would
 * otherwise do. Through a crossing our own lines are missing, so the corridor
 * gets short, the see-distance cue in driver.c reads that as "cannot see far
 * enough to go fast" and lifts; on a wide crossing the track is lost outright
 * and the failsafe stops the car. Recognised, the car holds the wheel where it
 * was and keeps its foot down.
 *
 * Pure arithmetic, no SDK, so it compiles and runs on a PC with the rest of the
 * control chain.
 */
#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "track.h"

typedef enum
{
    XSEC_IDLE = 0, /* nothing that looks like a crossing                 */
    XSEC_AHEAD,    /* recognised, still approaching it                   */
    XSEC_CROSSING, /* latched - driving through, on held steering        */
    XSEC_CLEAR     /* just came out of one, ignoring its far edge        */
} XsecPhase;

typedef struct
{
    XsecPhase phase;
    bool      recognised; /* this frame's raw verdict, before the latch   */
    bool      bothSides;  /* bars left and right of us - a real crossing  */
    bool      spanning;   /* one bar right across - a start/finish line   */
    uint8_t   bars;       /* near-sideways vectors in the winning group   */
    float     barY;       /* image row they sit on, 0 when there are none */
    float     cover;      /* their total width, in corridor widths        */
    float     gap;        /* hole between them, in corridor widths        */
    float     diverge;    /* far spread / near spread; >1 means splaying  */
    float     square;     /* 1 = a perfect right-angle elbow pair          */
    bool      camAgreed;  /* the camera's own detector backed this frame   */
    float     holdSteer;  /* steering angle frozen on the way in          */
    float     holdYaw;    /* 0..1, how much of it was a yaw correction    */
    float     steerOut;   /* what to steer this frame while holding       */
    float     runM;       /* metres travelled in the current phase        */
    float     seenM;      /* metres since a crossing was last recognised  */
    uint8_t   agree;      /* consecutive frames that agreed               */
    uint32_t  count;      /* crossings taken since the last reset         */
} XsecState;

void Xsec_Init(void);

/*
 * The Pixy2's own junction detector, handed over once per camera frame.
 *
 * The camera runs its own intersection finder over the WHOLE image, not over
 * the dozen vectors that survive to this driver, so it can see a junction whose
 * far corner never became a vector at all. That is precisely the case the
 * geometry here cannot settle on its own: one elbow, and no way to tell a
 * crossing with a missed corner from our own track turning.
 *
 * It is a tie-breaker and never a trigger. It can only unlock a reading the
 * geometry has already half-made; it can never conjure one out of a frame that
 * shows no elbow at all. Pass seen=false on any frame the camera reported none,
 * which is what makes a dropped frame fail closed.
 *
 * x and y are in the same 79 x 52 grid as the vectors. Call before Xsec_Update.
 * The host simulator never calls this - there is no Pixy2 firmware in it - so
 * every simulated result is the geometry alone, exactly as before.
 */
void Xsec_CameraHint(bool seen, float x, float y, uint8_t branches);

/*
 * Call once per camera frame, after Track_Update.
 *   segs / n   - the same raw vectors the track model was built from
 *   m          - the track model built from them
 *   travelM    - metres driven since the previous call
 *   steerNow   - the steering the controller wants, used as the hold value
 *   headNow    - the near heading that steering is answering, filtered the
 *                same way. How much of the held angle is a yaw correction -
 *                and so how much of it must be let go of inside the junction -
 *                is read off this.
 */
void Xsec_Update(const TrkSegment *segs, uint8_t n, const TrackModel *m,
                 float travelM, float steerNow, float headNow, XsecState *out);

/* True while the car should ignore what the corridor is doing and just go.
 * Steer s->steerOut while it is. */
bool Xsec_Holding(const XsecState *s);

/* True while the speed planner must not lift for a shrinking corridor. */
bool Xsec_KeepPower(const XsecState *s);

/* Room for the filtered copy below. Never fewer than the camera can send. */
#define XSEC_MAX_SEGS 16u

/*
 * The vectors the track model should be built from, given what the crossing
 * detector currently believes.
 *
 * While nothing is recognised this is every vector, untouched, so a lap with
 * no junction on it is bit for bit the lap it always was. From the moment a
 * crossing is recognised until the car is clear of it, vectors lying across
 * the frame are left out.
 *
 * They have to be. track.c drops a vector spanning fewer than
 * TRK_MIN_VECTOR_DY rows, on the grounds that it is a start line or a crossing
 * bar, and that is not enough: a crossing edge a metre ahead is drawn across
 * four or five rows while running thirty columns sideways, so it clears the
 * test. It then arrives as an ordinary track edge, and since it lies across
 * our corridor it is the innermost thing on its side and wins the row. The
 * corridor centre jumps onto the other track, the headings triple, and the car
 * steers off after it - which is also why it never gets to commit to the
 * crossing, because the latch refuses while the near heading says the road is
 * turning. Measured on a junction just past a bend: the car left the track by
 * more than a metre on this alone, and does not with the filter in.
 *
 * The same test cannot be made unconditional inside track.c. Our own edges
 * lie nearly as flat near the vanishing point in a tight corner - measured
 * across the five circuits, 6522 vectors in a clean lap are flatter than a
 * crossing edge needs to be - and throwing those away costs the corner.
 * Gating it on the detector is what makes it safe: it only applies where a
 * crossing has already been recognised, and there the flat vector is far more
 * likely to be the thing that was recognised.
 *
 * The detector itself always gets the unfiltered list, since those bars and
 * arms are exactly what it is looking for. Call with the state from the
 * previous frame, before Track_Update.
 */
uint8_t Xsec_ForTrack(const XsecState *s, const TrkSegment *in, uint8_t n,
                      TrkSegment *out);

#endif /* INTERSECTION_H */
