"""
robust.py - find the fastest tuning that is fast from ANYWHERE on the track.

optimize.py scores a tuning on five circuits, each driven from one starting
offset. That is one trajectory per layout, and a search will quietly spend every
centimetre of margin on the trajectories it is not being shown. Running the same
layouts from five offsets each turns 5 measurements into 60, and the shipped tune
loses the car on 12 of them - on layouts the old suite calls clean.

So a candidate here has to survive all of:

    tracks     12 layouts x 5 starting offsets, no line touched, none stopped
    circuits   the five original layouts
    faults     dropped frames and blind patches
    mountings  the 18 in-envelope camera aims

and only then is it ranked on time. Time is the total over all 60 track runs,
with a run that leaves the track charged the full time limit, so a tune cannot
buy pace by writing one off.

    python robust.py              grid the pace/margin knobs, then refine
    python robust.py --quick      the grid only
"""

import itertools
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CC = os.environ.get("CC", "gcc")
SRC = [os.path.join(HERE, "track_sim.c"),
       os.path.join(ROOT, "source", "track.c"),
       os.path.join(ROOT, "source", "racing_line.c"),
       os.path.join(ROOT, "source", "intersection.c"),
       os.path.join(ROOT, "source", "driver.c"),
       os.path.join(ROOT, "source", "speed_ctl.c")]
FIXED = ["-DRACE_BENCH_MODE=0"]


def evaluate(cfg, tag="rb"):
    exe = os.path.join(HERE, "rb_%s.exe" % tag)
    cmd = ([CC, "-O1", "-std=gnu99", "-o", exe] + SRC +
           ["-I" + os.path.join(ROOT, "include")] + FIXED +
           ["-D%s=%s" % kv for kv in cfg.items()] + ["-lm"])
    if subprocess.run(cmd, capture_output=True, text=True).returncode != 0:
        return None

    def go(*a):
        return subprocess.run([exe] + list(a), capture_output=True,
                              text=True).stdout

    t = go("-tracks")
    m = re.search(r"PART1 clean=(\d+)/(\d+) total=([0-9.]+) slowest=([0-9.]+)", t)
    if not m:
        return None
    clean, total, tsum, slow = (int(m.group(1)), int(m.group(2)),
                                float(m.group(3)), float(m.group(4)))

    out = go()
    laps = [float(x) for x in re.findall(r"laptime=\s*([0-9.]+)", out)][:5]
    exc = sum(int(x) for x in re.findall(r"excursions=(\d+)", out)[:5])
    dnf = out.count("*DNF*")

    faults = go("-fault").count("stayed inside the lines")

    e = go("-envelope")
    m = re.search(r"IN ENVELOPE\s*:\s*(\d+) configurations, \d+ finished, "
                  r"(\d+) with zero excursions", e)
    env_n, env_clean = (int(m.group(1)), int(m.group(2))) if m else (0, 0)

    try:
        os.remove(exe)
    except OSError:
        pass
    if len(laps) < 5:
        return None

    ok = (clean == total) and (exc == 0) and (dnf == 0) and \
         (faults >= GATE["faults"]) and (env_n == 18) and (env_clean >= GATE["env"])
    return {"clean": clean, "total": total, "tsum": tsum, "slow": slow,
            "circuits": sum(laps), "exc": exc, "dnf": dnf, "faults": faults,
            "env": env_clean, "pass": ok}


def show(label, r):
    if r is None:
        print("  %-46s  build failed" % label)
        return
    print("  %-46s tracks %2d/%-2d  %7.1fs  circuits %5.2fs  "
          "faults %d/8  env %2d/18  %s"
          % (label, r["clean"], r["total"], r["tsum"], r["circuits"],
             r["faults"], r["env"], "PASS" if r["pass"] else "fail"))
    sys.stdout.flush()


BASE = {}

# The shipped tune's scores on the two test sets a search would otherwise
# quietly spend. Filled in from the first evaluation, so a candidate has to be
# at least as good as what is already on the car - not perfect, which nothing is.
GATE = {"faults": 8, "env": 15}

GRID = {
    "SPEED_MAX":           ["100.0f", "90.0f", "80.0f"],
    "SPEED_MIN":           ["70.0f", "60.0f", "50.0f"],
    "SPEED_SEVERITY_FULL": ["1.15f", "0.95f", "0.75f"],
    "SAFE_MARGIN_FRAC":    ["0.22f", "0.28f", "0.34f"],
}

REFINE = [
    ("STEER_PP_SCALE",     ["1.15f", "1.45f", "1.60f"]),
    ("LINE_APEX_BIAS",     ["0.65f", "0.75f", "0.95f"]),
    ("SAFE_MARGIN_FRAC",   ["0.26f", "0.34f"]),
    ("SPEED_W_HEAD_FAR",   ["1.30f", "1.60f"]),
    ("SPEED_W_CURV",       ["1.10f", "1.40f"]),
    ("SPEED_SEE_FLOOR",    ["0.35f", "0.55f"]),
    ("STEER_KH",           ["20.0f", "40.0f"]),
    ("STEER_KD",           ["2.0f", "6.0f"]),
    ("SPEED_MIN",          ["50.0f", "62.0f", "72.0f"]),
    ("SPEED_MAX",          ["95.0f", "100.0f"]),
    ("SPEED_ACC_MS2",      ["2.20f", "4.00f"]),
    ("SPEED_DEC_MS2",      ["4.50f", "8.00f"]),
]


def main():
    print("=== the shipped tune ===")
    base = evaluate(BASE, "base")
    show("as shipped", base)
    if base:
        GATE["faults"] = base["faults"]
        GATE["env"] = base["env"]
        print("  gates: 60/60 track runs, circuits clean, faults >=%d, mountings >=%d"
              % (GATE["faults"], GATE["env"]))

    print("\n=== grid: pace against margin ===")
    best, bestr = None, None
    for combo in itertools.product(*GRID.values()):
        cfg = dict(zip(GRID.keys(), combo))
        r = evaluate(cfg)
        label = " ".join("%s=%s" % (k.split("_", 1)[1], v)
                         for k, v in cfg.items())
        if r is None:
            continue
        if r["pass"] or (r["clean"] >= 58):
            show(label, r)
        if r["pass"] and ((bestr is None) or (r["tsum"] < bestr["tsum"])):
            best, bestr = cfg, r

    if best is None:
        print("\n  nothing in the grid was clean from every start.")
        print("  best effort by clean count:")
        return 1

    print("\n  grid winner: %.1fs  %s" % (bestr["tsum"], best))
    if "--quick" in sys.argv:
        return 0

    print("\n=== refine: one knob at a time, keeping what helps ===")
    cur, curr = dict(best), bestr
    for key, values in REFINE:
        for v in values:
            trial = dict(cur)
            trial[key] = v
            rr = evaluate(trial)
            if rr and rr["pass"] and (rr["tsum"] < curr["tsum"] - 0.05):
                show("%s=%s  (kept, -%.2fs)" % (key, v, curr["tsum"] - rr["tsum"]), rr)
                cur, curr = trial, rr

    print("\n" + "=" * 78)
    print("FASTEST TUNE THAT IS CLEAN FROM EVERY START, ON EVERY LAYOUT")
    print("=" * 78)
    print("  60/60 track runs clean, %d/8 faults, %d/18 mountings"
          % (curr["faults"], curr["env"]))
    print("  %.1fs over the 60 runs (shipped: %.1fs, %d/60 clean)"
          % (curr["tsum"], base["tsum"] if base else 0.0,
             base["clean"] if base else 0))
    print("  %.2fs over the five original circuits (shipped: %.2fs)"
          % (curr["circuits"], base["circuits"] if base else 0.0))
    print()
    for k, v in sorted(cur.items()):
        print("  #define %-26s %s" % (k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
