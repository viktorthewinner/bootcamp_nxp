/*
 * features.h - turns one camera frame into the fixed vector a classifier can eat.
 *
 * THE PROBLEM THIS SOLVES
 *
 * The Pixy2 hands over up to twelve line segments in whatever order it found them.
 * A neural net needs a fixed number of inputs in a fixed order, so something has to
 * bridge the two - and the obvious bridge, "slot 0 is the first vector, slot 1 the
 * second", is the wrong one. The camera's ordering carries no meaning, so a model
 * fed that way has to learn from data that every permutation of twelve vectors means
 * the same thing. That is capacity spent on a symmetry that can simply be built in.
 *
 * So every feature here is either
 *
 *   - read out of the TrackModel, which track.c already canonicalised into eight
 *     rows ordered from the bumper outwards, or
 *   - an aggregate over the raw segments that does not depend on their order:
 *     counts, sums, maxima, and pairwise extremes.
 *
 * Reorder the input and the output is bit for bit identical. That is the whole point.
 *
 * WHY BOTH SOURCES AND NOT JUST THE TRACK MODEL
 *
 * Because at a crossing the track model is deliberately wrong. intersection.h spells
 * it out: track.c throws away the bars cutting across the road and reports a corridor
 * that opens out sideways, which is the symptom of a crossing rather than the evidence
 * for one. A classifier fed only the corridor would have to guess "crossing" from
 * "the road got wider", which is also what a lost edge looks like.
 *
 * The evidence lives in the raw segments, and it takes two forms:
 *
 *   HEAD ON   both edges stop at about the same distance, there is white space of
 *             roughly a track width, and short bars run across at the mouth.
 *
 *   AT AN ANGLE   the near edge and the crossing's own edge meet at a sharp vertex,
 *             and only part of the junction is in frame at all. The camera reports
 *             two long vectors sharing an endpoint with something near a right angle
 *             between them - which no smooth track edge ever produces.
 *
 * That second one is the case the geometry in intersection.c gives up on ("crossing
 * just after a bend" latches nothing), and it is why the vertex features below are
 * measured on the ground in centimetres rather than in the picture. A right angle on
 * the track is a right angle at any distance; in the image it is not.
 *
 * STATELESS ON PURPOSE
 *
 * Nothing here remembers anything. The frame goes in, the numbers come out, and two
 * identical frames always give identical features. History is real and it matters -
 * a crossing is an edge stopping and then resuming, which one frame cannot show - but
 * it is classifier.c that owns the ring buffer, so that the thing shared between the
 * training set and the car has no hidden state to drift.
 *
 * No SDK, no hardware, no file scope state - same rule as track.c, and for the same
 * reason: this exact code has to run in the trainer and on the car, or the model is
 * being fed something different from what it learned on.
 */
#ifndef FEATURES_H
#define FEATURES_H

#include <stdint.h>
#include <stdbool.h>
#include "track.h"

/* Most segments one frame can contain. This is PIXY_MAX_VECTORS, but spelled
 * again here rather than included: pixy.h reaches into the SDK, and this file has
 * to compile on the host trainer as well as on the car. A caller handing over
 * more than this is clamped, not rejected. */
#define FEAT_MAX_SEG 12

/* Layout of the feature vector. The indices are named because the training script
 * prints them by name when it reports which inputs the model actually leaned on. */
enum
{
    /* --- corridor shape, straight out of the TrackModel --- */
    FEAT_CENTER0 = 0,                       /* 8: corridor middle, per row, normalised */
    FEAT_WIDTH0  = FEAT_CENTER0 + TRK_ROWS, /* 8: corridor width, per row              */
    FEAT_VALID0  = FEAT_WIDTH0  + TRK_ROWS, /* 8: was this row measured at all         */

    /* --- where the road is pointing --- */
    FEAT_HEAD_NEAR = FEAT_VALID0 + TRK_ROWS,
    FEAT_HEAD_FAR,
    FEAT_CURV,
    FEAT_ABS_HEAD_FAR,  /* corners are symmetric; sign only tells you left from right */
    FEAT_ABS_CURV,
    FEAT_NVALID,

    /* --- order free aggregates over the raw segments --- */
    FEAT_NSEG,          /* how many the camera reported                         */
    FEAT_MEAN_LEN,
    FEAT_MAX_LEN,
    FEAT_N_CROSS,       /* segments lying across the road, not up it            */
    FEAT_CROSS_LEN,     /* total length of them - the bars at a crossing mouth  */
    FEAT_N_UP,
    FEAT_SLOPE_SPREAD,  /* up-track segments disagreeing about which way to go  */
    FEAT_TOP_REACH,     /* how far ahead the frame still shows a line           */
    FEAT_FAR_SPREAD,    /* lateral spread of the far endpoints                  */

    /* --- the crossing signature, measured on the ground --- */
    FEAT_VERTEX_MAX,    /* sharpest corner formed by two segments meeting       */
    FEAT_VERTEX_N,
    FEAT_GAP_LEFT,      /* longest hole in the left edge, cm                    */
    FEAT_GAP_RIGHT,
    FEAT_STOP_SKEW,     /* do both edges stop at the same distance              */
    FEAT_STOP_FWD,      /* and how far ahead is that                            */
    FEAT_BAR_BEYOND,    /* is there something lying across at or past the stop  */
    FEAT_WIDTH_BLOWUP,  /* widest row over typical row - the corridor opening   */
    FEAT_SAW_ASYM,      /* one edge seen far more than the other                */

    FEAT_N              /* = 48 */
};

/* The scalars classifier.c keeps a short history of. A crossing is a sequence,
 * not a picture: the edge stops, there is white, the edge resumes. These are the
 * eight numbers whose recent past says most about which of those is happening. */
enum
{
    FHIST_HEAD_FAR = 0,
    FHIST_CURV,
    FHIST_VERTEX_MAX,
    FHIST_CROSS_LEN,
    FHIST_GAP_LEFT,
    FHIST_GAP_RIGHT,
    FHIST_WIDTH_BLOWUP,
    FHIST_NVALID,

    FHIST_N             /* = 8 */
};

/*
 * Builds the feature vector for one frame.
 *
 * segs/n are the RAW vectors, exactly as handed to Track_Update and
 * Intersection_Update - not the filtered set. The bars that track.c discards are
 * evidence here, so they must still be in the list.
 *
 * tm is the model track.c produced from those same vectors this frame.
 *
 * out must have room for FEAT_N floats. Every one is written, so there is no need
 * to clear it first, and every one is finite whatever the camera did - including
 * n == 0, which is a legitimate frame meaning "I can see nothing".
 */
void Features_Build(const TrkSegment *segs, uint8_t n, const TrackModel *tm,
                    float *out);

/*
 * Copies the FHIST_N history scalars out of a finished feature vector.
 * classifier.c calls this rather than recomputing anything.
 */
void Features_History(const float *feat, float *out);

/* Human readable name for one feature index, for the training report and for
 * anything that wants to print a frame. Never returns NULL. */
const char *Features_Name(int idx);

#endif /* FEATURES_H */
