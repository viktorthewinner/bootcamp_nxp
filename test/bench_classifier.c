/*
 * bench_classifier.c - what one frame of classification actually costs.
 *
 * Times the whole per frame path, not just the matrix multiplies: unprojecting
 * the segments, the pairwise vertex scan, the corridor statistics, the ring
 * buffer, three dense layers and the softmax. That is what the car pays, so that
 * is what is measured.
 *
 * The host figure is not the answer for the M33 - different core, different
 * clock, different FPU - but it does establish the shape: how the cost divides
 * between building the features and running the network, and whether anything in
 * here is accidentally expensive. The M33 estimate is derived from the operation
 * count and printed alongside.
 *
 *   gcc -O2 -std=gnu99 -o bench test/bench_classifier.c source/features.c \
 *       source/classifier.c source/track.c -Iinclude -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "classifier.h"
#include "features.h"
#include "track.h"

#define ITERS 200000

/*
 * A frame shaped like the hard case: the car arriving at a crossing off the exit
 * of a bend, so both edges are in shot, one of them is broken by the junction,
 * and two of the crossing's own bars are visible at an angle. Eight vectors,
 * which is about what the Pixy2 reports on a busy frame.
 */
static const TrkSegment s_frame[8] = {
    { 22.0f, 51.0f, 27.0f, 30.0f },   /* left edge, near piece            */
    { 27.0f, 30.0f, 29.0f, 19.0f },   /* left edge, continuing            */
    { 56.0f, 51.0f, 49.0f, 31.0f },   /* right edge, near piece           */
    { 49.0f, 31.0f, 46.0f, 22.0f },   /* right edge, stops at the mouth   */
    { 29.0f, 19.0f, 14.0f, 17.0f },   /* bar running out to the left      */
    { 46.0f, 22.0f, 63.0f, 20.0f },   /* bar running out to the right     */
    { 31.0f, 14.0f, 33.0f,  9.0f },   /* far piece beyond the crossing    */
    { 45.0f, 13.0f, 44.0f,  9.0f },
};

static void fake_model(TrackModel *tm)
{
    int r;

    (void)memset(tm, 0, sizeof(*tm));
    for (r = 0; r < TRK_ROWS; r++)
    {
        tm->y[r]      = 51.0f - (float)r * 5.0f;
        tm->xl[r]     = 22.0f + (float)r * 1.1f;
        tm->xr[r]     = 56.0f - (float)r * 1.4f;
        tm->center[r] = (tm->xl[r] + tm->xr[r]) * 0.5f;
        tm->width[r]  = tm->xr[r] - tm->xl[r];
        tm->margin[r] = 3.0f;
        tm->valid[r]  = true;
        tm->sawL[r]   = (r < 6);
        tm->sawR[r]   = (r < 4);
    }
    tm->segCount = 8u;
    tm->nValid   = TRK_ROWS;
    tm->topRow   = TRK_ROWS - 1u;
    tm->haveTrack = true;
    tm->bothEdges = true;
    tm->headNear  = 0.18f;
    tm->headFar   = 0.62f;
    tm->curv      = 0.44f;
}

int main(void)
{
    Classifier net;
    TrackModel tm;
    float      feat[FEAT_N];
    clock_t    t0;
    double     secs, nsFeat, nsAll;
    long       i;
    volatile float sink = 0.0f;

    Track_Init();
    Classifier_Init(&net);
    fake_model(&tm);

    /* ---- features only ---- */
    t0 = clock();
    for (i = 0; i < ITERS; i++)
    {
        Features_Build(s_frame, 8u, &tm, feat);
        sink += feat[0];
    }
    secs   = (double)(clock() - t0) / (double)CLOCKS_PER_SEC;
    nsFeat = secs * 1e9 / (double)ITERS;

    /* ---- the whole path ---- */
    t0 = clock();
    for (i = 0; i < ITERS; i++)
    {
        (void)Classifier_Step(&net, s_frame, 8u, &tm);
        sink += net.prob[0];
    }
    secs  = (double)(clock() - t0) / (double)CLOCKS_PER_SEC;
    nsAll = secs * 1e9 / (double)ITERS;

    printf("=== cost of one classified frame ===\n\n");
    printf("network        %d -> %d -> %d -> %d\n",
           NET_IN, NET_H1, NET_H2, NET_OUT);
    {
        long macs = (long)NET_IN * NET_H1 + (long)NET_H1 * NET_H2
                  + (long)NET_H2 * NET_OUT;
        long prm  = macs + NET_H1 + NET_H2 + NET_OUT;

        printf("weights        %ld MACs, %ld parameters, %.1f KB fp32\n\n",
               macs, prm, (double)prm * 4.0 / 1024.0);

        printf("host, x86 -O2\n");
        printf("  features     %7.2f us\n", nsFeat / 1000.0);
        printf("  + network    %7.2f us   (network alone %.2f us)\n",
               nsAll / 1000.0, (nsAll - nsFeat) / 1000.0);
        printf("\n");

        /*
         * The M33 has a single precision FPU with a fused multiply-add that
         * issues once per cycle, so the dense layers are MAC bound and the count
         * is the cycle count to within loop overhead. Three expf calls for the
         * softmax are the only expensive scalar operations left; newlib's costs
         * a few hundred cycles each on this core.
         */
        printf("MCXN947 estimate, M33 at 150 MHz with FPv5-SP\n");
        printf("  dense layers %7.2f us   (%ld VFMA at 1/cycle)\n",
               (double)macs / 150.0, macs);
        printf("  softmax      %7.2f us   (3 x expf)\n", 900.0 / 150.0);
        printf("  features     %7.2f us   (scaled from host by MAC ratio)\n",
               nsFeat / 1000.0 * 3.0);
        printf("  ------------------------\n");
        printf("  per frame  ~ %7.2f us   of a %.1f ms budget at 60 fps\n",
               (double)macs / 150.0 + 6.0 + nsFeat / 1000.0 * 3.0, 16.7);
    }

    if (sink == 12345.678f) printf("");   /* keep the loops from being elided */
    return 0;
}
