/*
 * classifier.h - is the road ahead straight, a corner, or a junction?
 *
 * A small fully connected network over the features features.c builds, with a few
 * frames of history stacked on. It answers the one question the geometry in
 * intersection.c is weakest at: the crossing approached at an angle, where the
 * camera never sees the mouth square on and reports one corner of the junction
 * instead of a hole with parallel sides.
 *
 * WHY THERE IS HISTORY IN HERE AT ALL
 *
 * Because a single frame genuinely cannot settle it, and intersection.h says so:
 * both edges stopping a short way ahead "is also what a camera that simply cannot
 * see very far reports in an ordinary corner". What separates the two is what
 * happens next - the edge stops, there is white, the edge resumes - and that is a
 * sequence. Hand-writing it cost intersection.c a latch, a phase enum and a
 * confirmation counter. Here it is eight numbers from three earlier frames, and
 * the training set teaches the shape.
 *
 * WHAT IT COSTS
 *
 * About 2,200 multiply-accumulates. On the M33 at 150 MHz with the hard float unit
 * that is roughly 20 microseconds - near enough a thousandth of the 16.7 ms a
 * camera frame gives you. There is no quantisation and no NPU here on purpose:
 * an int8 build would need SIMD intrinsics to be any quicker, and the thing it
 * would be making quicker already costs nothing.
 *
 * WHAT TO DO WITH THE ANSWER
 *
 * Not steer by it directly. intersection.c stays the floor - it is thirty
 * kilobytes of geometry that fails in ways a telemetry dump can explain, and a
 * network is not. Use this to arbitrate: to confirm what the geometry already
 * suspects, and to catch the oblique case the geometry drives straight past.
 * The probabilities are exposed rather than just the winning class precisely so
 * the caller can require agreement, or a margin, before acting.
 *
 * No SDK, no hardware, no file scope state - the caller owns the struct, so the
 * simulator can run this exactly as the car does.
 */
#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <stdint.h>
#include <stdbool.h>
#include "track.h"
#include "features.h"
#include "net_weights.h"

typedef enum
{
    CLS_STRAIGHT     = 0,
    CLS_CORNER       = 1,
    CLS_INTERSECTION = 2,
    CLS_COUNT        = 3
} ClassId;

typedef struct
{
    /* Ring of the history scalars, newest at head. One slot more than the
     * deepest lag, so the oldest frame the network asks for is still here. */
    float   ring[NET_MAXLAG + 1][FHIST_N];
    uint8_t head;
    uint8_t filled;

    /* ---- the latest answer ---- */
    ClassId cls;              /* the winning class                          */
    float   prob[CLS_COUNT];  /* softmax, sums to 1                         */
    float   margin;           /* winner minus runner up, before the softmax.
                               * Cheaper to threshold on than a probability
                               * and monotonic in the same direction.       */
    bool    ready;            /* false until enough frames have gone by for
                               * the history to be real rather than padded  */
} Classifier;

/* Clears the ring and the last answer. Call once, from Driver_Init. */
void Classifier_Init(Classifier *c);

/*
 * Call once per camera frame, with the RAW vectors - the same array handed to
 * Track_Update and Intersection_Update - and the model track.c built from them.
 *
 * Returns the winning class and leaves the probabilities in the struct. Safe to
 * call on a frame with no vectors at all; the answer will simply be uncertain.
 */
ClassId Classifier_Step(Classifier *c, const TrkSegment *segs, uint8_t n,
                        const TrackModel *tm);

/* Name for a class, for telemetry and the simulator's reports. */
const char *Classifier_Name(ClassId c);

#endif /* CLASSIFIER_H */
