"""
Trains the straight / corner / intersection classifier and writes the C header
the firmware compiles in.

    python test/train_classifier.py <train.bin> <val.bin> [out.h]

Pure numpy on purpose. The network is about two thousand multiply-accumulates;
pulling in a training framework to fit that would add a large dependency, hide the
arithmetic behind an abstraction, and change nothing about the answer. Everything
here - forward, backward, Adam - is a dozen lines each and can be read.

WHAT COMES OUT

A header with the weights already folded together with the input normalisation, so
the car does no standardising at run time:

    z = W @ ((x - mean) / std) + b   becomes   z = W' @ x + b'
    W' = W / std                               b' = b - W' @ mean

Same numbers, two fewer passes over the input vector, and nothing on the car has to
know what the training set's statistics were.

HOW IT IS EVALUATED

Never on rows from a circuit it trained on. The two files come from different
generator seeds, so no layout, no camera mounting and no crossing position is
shared. Splitting rows at random instead would leak badly: consecutive frames of
one lap are nearly the same picture, so a random split puts near-duplicates on both
sides and reports a score that has nothing to do with a new track.

Accuracy is not reported as a single number either. Intersections are a small
minority of frames, so "always say straight" already scores in the seventies. What
matters is per class recall and precision, and those are what the report prints.
"""

import sys

import numpy as np

import mldata

LAGS = (2, 4, 8)          # camera frames back; at ~60 fps that is 33, 67, 133 ms
H1, H2 = 32, 16
EPOCHS = 60
BATCH = 512
LR = 3e-3
SEED = 12345
CLASS_W_POWER = 0.75      # 0 = ignore imbalance, 1 = full inverse frequency


# ----------------------------------------------------------------- data

def build_inputs(ds):
    """Current features plus the history scalars at each lag.

    The lag is taken within the track: frame 0 of a circuit has no past, so it
    looks back at itself rather than at the last frame of the previous circuit,
    which would be a different track seen through a different camera.
    """
    feat = ds.feat
    n, f = feat.shape

    hist_cols = [mldata.FEATURE_NAMES.index(nm) for nm in mldata.HIST_NAMES]
    hist = feat[:, hist_cols]

    # First row index of the track each row belongs to.
    starts = np.zeros(n, dtype=np.int64)
    change = np.flatnonzero(np.diff(ds.track) != 0) + 1
    bounds = np.concatenate(([0], change, [n]))
    for a, b in zip(bounds[:-1], bounds[1:]):
        starts[a:b] = a

    idx = np.arange(n)
    parts = [feat]
    for lag in LAGS:
        parts.append(hist[np.maximum(idx - lag, starts)])

    return np.ascontiguousarray(np.concatenate(parts, axis=1), dtype=np.float32)


def input_names():
    names = list(mldata.FEATURE_NAMES)
    for lag in LAGS:
        names += [f"{nm}@-{lag}" for nm in mldata.HIST_NAMES]
    return names


# ----------------------------------------------------------------- model

class MLP:
    def __init__(self, nin, rng):
        def he(a, b):
            return (rng.standard_normal((a, b)) * np.sqrt(2.0 / b)).astype(np.float32)

        self.W = [he(H1, nin), he(H2, H1), he(3, H2)]
        self.b = [np.zeros(H1, np.float32), np.zeros(H2, np.float32),
                  np.zeros(3, np.float32)]
        self.m = [np.zeros_like(p) for p in self.W + self.b]
        self.v = [np.zeros_like(p) for p in self.W + self.b]
        self.t = 0

    def forward(self, x):
        a1 = np.maximum(x @ self.W[0].T + self.b[0], 0.0)
        a2 = np.maximum(a1 @ self.W[1].T + self.b[1], 0.0)
        return a1, a2, a2 @ self.W[2].T + self.b[2]

    def step(self, grads, lr):
        self.t += 1
        params = self.W + self.b
        for i, (p, g) in enumerate(zip(params, grads)):
            self.m[i] = 0.9 * self.m[i] + 0.1 * g
            self.v[i] = 0.999 * self.v[i] + 0.001 * (g * g)
            mh = self.m[i] / (1.0 - 0.9 ** self.t)
            vh = self.v[i] / (1.0 - 0.999 ** self.t)
            p -= lr * mh / (np.sqrt(vh) + 1e-8)


def softmax(z):
    z = z - z.max(axis=1, keepdims=True)
    e = np.exp(z)
    return e / e.sum(axis=1, keepdims=True)


def train_batch(net, x, y, w, lr):
    a1, a2, z = net.forward(x)
    p = softmax(z)

    n = len(y)
    loss = -np.log(np.maximum(p[np.arange(n), y], 1e-9))
    loss = float((loss * w).sum() / w.sum())

    dz = p.copy()
    dz[np.arange(n), y] -= 1.0
    dz *= (w / w.sum())[:, None]

    gW2 = dz.T @ a2
    gb2 = dz.sum(0)
    da2 = (dz @ net.W[2]) * (a2 > 0)
    gW1 = da2.T @ a1
    gb1 = da2.sum(0)
    da1 = (da2 @ net.W[1]) * (a1 > 0)
    gW0 = da1.T @ x
    gb0 = da1.sum(0)

    net.step([gW0, gW1, gW2, gb0, gb1, gb2], lr)
    return loss


# ----------------------------------------------------------------- report

def report(y, pred):
    k = len(mldata.CLASS_NAMES)
    cm = np.zeros((k, k), dtype=np.int64)
    for t, p in zip(y, pred):
        cm[t, p] += 1

    lines = ["            " + "".join(f"{n:>14s}" for n in mldata.CLASS_NAMES)
             + "     recall"]
    f1s = []
    for i, name in enumerate(mldata.CLASS_NAMES):
        rec = cm[i, i] / max(cm[i].sum(), 1)
        prec = cm[i, i] / max(cm[:, i].sum(), 1)
        f1s.append(0.0 if (prec + rec) == 0 else 2 * prec * rec / (prec + rec))
        lines.append(f"  {name:10s}" + "".join(f"{v:14d}" for v in cm[i])
                     + f"{100 * rec:10.1f}%")
    lines.append("  precision " + "".join(
        f"{100 * cm[i, i] / max(cm[:, i].sum(), 1):13.1f}%" for i in range(k)))
    lines.append("")
    for name, f1 in zip(mldata.CLASS_NAMES, f1s):
        lines.append(f"  F1 {name:13s} {f1:.4f}")
    macro = float(np.mean(f1s))
    lines.append(f"  macro F1        {macro:.4f}")
    return macro, "\n".join(lines)


# ----------------------------------------------------------------- export

HEADER = '''/*
 * net_weights.h - GENERATED by test/train_classifier.py. Do not edit.
 *
 * {rows} training frames from {tracks} generated circuits ({oblique} of them with
 * the crossing just past a corner exit), validated on {vrows} frames from {vtracks}
 * circuits the model never saw.
 *
 *   macro F1 {macro:.4f}   intersection recall {irec:.1f}%   precision {iprec:.1f}%
 *
 * The input normalisation is folded into the first layer, so the car feeds the
 * feature vector in raw. {nin} inputs -> {h1} -> {h2} -> 3.
 */
#ifndef NET_WEIGHTS_H
#define NET_WEIGHTS_H

#define NET_IN   {nin}
#define NET_H1   {h1}
#define NET_H2   {h2}
#define NET_OUT  3

/* How many camera frames back the history scalars are taken from. */
#define NET_LAGS   {{ {lags} }}
#define NET_NLAG   {nlag}
#define NET_MAXLAG {maxlag}

'''


def emit_matrix(name, m):
    rows, cols = m.shape
    out = [f"static const float {name}[{rows}][{cols}] = {{"]
    for r in range(rows):
        vals = ", ".join(f"{v: .6e}f" for v in m[r])
        out.append(f"    {{ {vals} }},")
    out.append("};\n")
    return "\n".join(out)


def emit_vector(name, v):
    vals = ", ".join(f"{x: .6e}f" for x in v)
    return f"static const float {name}[{len(v)}] = {{ {vals} }};\n"


def export(path, net, mean, std, meta):
    # Fold standardisation into layer 1: z = W((x-mean)/std)+b = (W/std)x + (b - W(mean/std))
    w0 = net.W[0] / std[None, :]
    b0 = net.b[0] - w0 @ mean

    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(HEADER.format(nin=net.W[0].shape[1], h1=H1, h2=H2,
                               lags=", ".join(str(l) for l in LAGS),
                               nlag=len(LAGS), maxlag=max(LAGS), **meta))
        fh.write(emit_matrix("NET_W1", w0))
        fh.write(emit_vector("NET_B1", b0))
        fh.write(emit_matrix("NET_W2", net.W[1]))
        fh.write(emit_vector("NET_B2", net.b[1]))
        fh.write(emit_matrix("NET_W3", net.W[2]))
        fh.write(emit_vector("NET_B3", net.b[2]))
        fh.write("\n#endif /* NET_WEIGHTS_H */\n")


# ----------------------------------------------------------------- main

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1

    out_h = sys.argv[3] if len(sys.argv) > 3 else "include/net_weights.h"

    tr = mldata.load(sys.argv[1])
    va = mldata.load(sys.argv[2])
    print("train:", tr.summary(), sep="\n")
    print("\nval:  ", va.summary(), sep="\n")

    xtr, ytr = build_inputs(tr), tr.label
    xva, yva = build_inputs(va), va.label
    print(f"\n{xtr.shape[1]} inputs "
          f"({tr.feat.shape[1]} this frame + {len(mldata.HIST_NAMES)} x {len(LAGS)} lagged)")

    mean = xtr.mean(0)
    std = xtr.std(0)
    std[std < 1e-6] = 1.0
    xtr_n = (xtr - mean) / std
    xva_n = (xva - mean) / std

    freq = np.bincount(ytr, minlength=3).astype(np.float64)
    cw = (freq.sum() / np.maximum(freq, 1)) ** CLASS_W_POWER
    cw /= cw.mean()
    print("class weights: " + ", ".join(
        f"{n}={w:.2f}" for n, w in zip(mldata.CLASS_NAMES, cw)))

    rng = np.random.default_rng(SEED)
    net = MLP(xtr_n.shape[1], rng)
    wtr = cw[ytr].astype(np.float32)

    best, best_state, best_txt = -1.0, None, ""
    n = len(ytr)
    print(f"\ntraining {EPOCHS} epochs over {n} frames")

    for ep in range(EPOCHS):
        order = rng.permutation(n)
        lr = LR * (0.5 ** (ep / 20.0))
        tot = 0.0
        for a in range(0, n, BATCH):
            sel = order[a:a + BATCH]
            tot += train_batch(net, xtr_n[sel], ytr[sel], wtr[sel], lr)

        pred = net.forward(xva_n)[2].argmax(1)
        macro, txt = report(yva, pred)
        if macro > best:
            best = macro
            best_txt = txt
            best_state = ([w.copy() for w in net.W], [b.copy() for b in net.b])

        if ep % 5 == 0 or ep == EPOCHS - 1:
            print(f"  epoch {ep:3d}  loss {tot / (n / BATCH):.4f}  "
                  f"val macro F1 {macro:.4f}{'  *' if macro >= best else ''}")

    net.W, net.b = best_state
    print("\n=== best model, on circuits it never trained on ===")
    print(best_txt)

    pred = net.forward(xva_n)[2].argmax(1)
    cm = np.zeros((3, 3), np.int64)
    for t, p in zip(yva, pred):
        cm[t, p] += 1
    irec = 100 * cm[2, 2] / max(cm[2].sum(), 1)
    iprec = 100 * cm[2, 2] / max(cm[:, 2].sum(), 1)

    export(out_h, net, mean, std, dict(
        rows=len(tr), tracks=tr.meta["tracks"], oblique=tr.meta["oblique"],
        vrows=len(va), vtracks=va.meta["tracks"],
        macro=best, irec=irec, iprec=iprec))

    macs = xtr.shape[1] * H1 + H1 * H2 + H2 * 3
    params = macs + H1 + H2 + 3
    print(f"\nwrote {out_h}")
    print(f"  {params} parameters, {macs} MACs, {params * 4 / 1024:.1f} KB as fp32")
    return 0


if __name__ == "__main__":
    sys.exit(main())
