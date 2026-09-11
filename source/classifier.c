/*
 * classifier.c - the forward pass. See classifier.h for what it is for.
 *
 * Three dense layers, ReLU between them, softmax at the end. That is the whole
 * network, and it is written out longhand rather than pulled from a runtime for
 * two reasons: it keeps this file free of SDK and framework dependencies, so the
 * simulator compiles it exactly as the car does; and at this size a library would
 * spend more cycles deciding what to do than doing it.
 *
 * The input normalisation is not here. It is folded into the first layer's
 * weights by the trainer, so the feature vector goes in raw - see the header
 * test/train_classifier.py writes.
 */
#include "classifier.h"

#include <math.h>
#include <string.h>

static const uint8_t s_lag[NET_NLAG] = NET_LAGS;

/*
 * The input vector is this frame's features followed by the history scalars at
 * each lag, so the weights only line up with the features if the shapes agree.
 * Regenerate net_weights.h after changing FEAT_N or LAGS and this stays true;
 * forget to, and without this line the mismatch would be a silent stack overrun
 * writing past in[] rather than a compile error.
 */
typedef char cls_input_shape_matches[
    (NET_IN == (FEAT_N + (NET_NLAG * FHIST_N))) ? 1 : -1];

/*
 * out = relu(W.in + b), with W stored row per output so the inner loop walks
 * memory forwards. Written to be trivially recognisable to the compiler as a
 * fused multiply-add chain: on the M33 with FPv5 this comes out as a VFMA per
 * weight, which is where the twenty microsecond figure in the header comes from.
 */
static void dense(const float *w, const float *b, const float *in,
                  int nOut, int nIn, float *out, bool relu)
{
    int o, i;

    for (o = 0; o < nOut; o++)
    {
        const float *row = w + ((size_t)o * (size_t)nIn);
        float        acc = b[o];

        for (i = 0; i < nIn; i++)
        {
            acc += row[i] * in[i];
        }

        if (relu && (acc < 0.0f))
        {
            acc = 0.0f;
        }
        out[o] = acc;
    }
}

void Classifier_Init(Classifier *c)
{
    (void)memset(c, 0, sizeof(*c));
    c->cls          = CLS_STRAIGHT;
    c->prob[CLS_STRAIGHT] = 1.0f;
}

ClassId Classifier_Step(Classifier *c, const TrkSegment *segs, uint8_t n,
                        const TrackModel *tm)
{
    float in[NET_IN];
    float h1[NET_H1];
    float h2[NET_H2];
    float z[NET_OUT];
    float top, sum;
    int   i, k, best;

    /* ---- this frame ----
     *
     * Built straight into the front of the input vector rather than into a
     * buffer of its own and copied. This is not micro-optimisation: the deepest
     * call chain on the car runs main -> Driver_Step -> here -> Features_Build,
     * and the stack is 4 KB, so a spare 192 byte frame on that path is worth
     * more than the line of code it saves. The copy went with it. */
    Features_Build(segs, n, tm, in);

    c->head = (uint8_t)((c->head + 1u) % (uint8_t)(NET_MAXLAG + 1));
    Features_History(in, c->ring[c->head]);
    if (c->filled <= (uint8_t)NET_MAXLAG)
    {
        c->filled++;
    }
    c->ready = (c->filled > (uint8_t)NET_MAXLAG);

    /* ---- then the history at each lag ----
     *
     * Before the ring has filled, a lag that reaches past the start reads the
     * oldest frame there is rather than a zero. Zeros would be a story about the
     * track - "there was nothing there a moment ago" - and the first crossing of
     * a run is not the moment to be telling the network that. */
    for (k = 0; k < NET_NLAG; k++)
    {
        int lag = (int)s_lag[k];
        int idx;

        if (lag > (int)c->filled - 1)
        {
            lag = (int)c->filled - 1;
        }
        if (lag < 0)
        {
            lag = 0;
        }

        idx = ((int)c->head - lag + (NET_MAXLAG + 1)) % (NET_MAXLAG + 1);
        (void)memcpy(&in[FEAT_N + (k * FHIST_N)], c->ring[idx],
                     sizeof(float) * (size_t)FHIST_N);
    }

    /* ---- forward ---- */
    dense(&NET_W1[0][0], NET_B1, in, NET_H1, NET_IN,  h1, true);
    dense(&NET_W2[0][0], NET_B2, h1, NET_H2, NET_H1,  h2, true);
    dense(&NET_W3[0][0], NET_B3, h2, NET_OUT, NET_H2, z,  false);

    /* ---- softmax, and the margin before it ---- */
    best = 0;
    for (i = 1; i < NET_OUT; i++)
    {
        if (z[i] > z[best])
        {
            best = i;
        }
    }

    top = z[best];
    c->margin = 1e30f;
    for (i = 0; i < NET_OUT; i++)
    {
        if (i != best)
        {
            float d = top - z[i];
            if (d < c->margin)
            {
                c->margin = d;
            }
        }
    }

    sum = 0.0f;
    for (i = 0; i < NET_OUT; i++)
    {
        c->prob[i] = expf(z[i] - top);   /* shifted, so nothing can overflow */
        sum += c->prob[i];
    }
    for (i = 0; i < NET_OUT; i++)
    {
        c->prob[i] /= sum;
    }

    c->cls = (ClassId)best;
    return c->cls;
}

const char *Classifier_Name(ClassId c)
{
    static const char *const names[CLS_COUNT] = {
        "straight", "corner", "intersection"
    };

    return ((int)c >= 0 && (int)c < CLS_COUNT) ? names[c] : "?";
}
