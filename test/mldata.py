"""
Reader for the dataset test/track_sim.c -mldata writes.

File layout, little endian throughout:

    uint32  magic 'TML1'
    uint32  version
    uint32  featN        floats per row
    uint32  histN        scalars classifier.c stacks
    uint32  rows
    uint32  tracks
    uint32  oblique      how many had the crossing just past a corner exit
    uint32  seed
    then rows x (uint16 track, uint16 frame, uint8 label, float32 feat[featN])

The per row header is five bytes and the features follow unaligned, so the whole
thing is read as one structured dtype rather than reshaped - numpy handles the
packing, and getting it wrong would silently shift every feature by a byte.
"""

import numpy as np

CLASS_NAMES = ["straight", "corner", "intersection"]

# Kept in step with include/features.h. The trainer prints these, and a mismatch
# would mislabel every column in the report without changing any number.
FEATURE_NAMES = (
    [f"center{i}" for i in range(8)]
    + [f"width{i}" for i in range(8)]
    + [f"valid{i}" for i in range(8)]
    + [
        "headNear", "headFar", "curv", "absHeadFar", "absCurv", "nValid",
        "nSeg", "meanLen", "maxLen", "nCross", "crossLen", "nUp",
        "slopeSpread", "topReach", "farSpread",
        "vertexMax", "vertexN", "gapLeft", "gapRight",
        "stopSkew", "stopFwd", "barBeyond", "widthBlowup", "sawAsym",
    ]
)

HIST_NAMES = [
    "headFar", "curv", "vertexMax", "crossLen",
    "gapLeft", "gapRight", "widthBlowup", "nValid",
]


class Dataset:
    def __init__(self, track, frame, label, feat, meta):
        self.track = track
        self.frame = frame
        self.label = label
        self.feat = feat
        self.meta = meta

    def __len__(self):
        return len(self.label)

    def summary(self):
        n = len(self.label)
        out = [
            f"{n} rows from {self.meta['tracks']} tracks "
            f"({self.meta['oblique']} oblique), {self.feat.shape[1]} features"
        ]
        for c, name in enumerate(CLASS_NAMES):
            k = int((self.label == c).sum())
            out.append(f"  {name:13s} {k:8d}  {100.0 * k / n:5.1f}%")
        return "\n".join(out)


def load(path):
    with open(path, "rb") as fh:
        head = np.frombuffer(fh.read(32), dtype="<u4")
        if head[0] != 0x314C4D54:
            raise ValueError(f"{path}: not a TML1 dataset (magic {head[0]:#x})")

        featN = int(head[2])
        meta = {
            "version": int(head[1]),
            "featN": featN,
            "histN": int(head[3]),
            "rows": int(head[4]),
            "tracks": int(head[5]),
            "oblique": int(head[6]),
            "seed": int(head[7]),
        }

        row = np.dtype(
            [("track", "<u2"), ("frame", "<u2"), ("label", "u1"),
             ("feat", "<f4", (featN,))]
        )
        raw = np.fromfile(fh, dtype=row)

    if meta["rows"] and len(raw) != meta["rows"]:
        raise ValueError(
            f"{path}: header says {meta['rows']} rows, found {len(raw)}"
        )

    return Dataset(
        raw["track"].astype(np.int32),
        raw["frame"].astype(np.int32),
        raw["label"].astype(np.int64),
        np.ascontiguousarray(raw["feat"].astype(np.float32)),
        meta,
    )


if __name__ == "__main__":
    import sys

    ds = load(sys.argv[1])
    print(ds.summary())
    print()

    names = FEATURE_NAMES[: ds.feat.shape[1]]
    print(f"{'feature':14s} {'mean':>9s} {'std':>9s} "
          + " ".join(f"{n:>10s}" for n in CLASS_NAMES))
    for i, name in enumerate(names):
        col = ds.feat[:, i]
        per = [col[ds.label == c].mean() if (ds.label == c).any() else float("nan")
               for c in range(3)]
        print(f"{name:14s} {col.mean():9.3f} {col.std():9.3f} "
              + " ".join(f"{v:10.3f}" for v in per))
