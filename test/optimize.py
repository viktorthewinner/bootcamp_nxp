"""
optimize.py - find the fastest tuning that will still be fast on the real car.

The distinction matters. The fastest tuning in the simulator is not the fastest
tuning on a track, because the simulator's camera is bolted exactly where the
nominal model says it is, and yours is not. A search that scores only lap time
will spend that robustness without telling you: two tunings that both look clean
on the circuits can be 18/18 and 15/18 across the sane camera mountings.

So every candidate here has to pass all three, and only then is it ranked on
time:

    circuits   five layouts, no line touched
    faults     a sixth layout, dropped frames, blind patches - all 8 clean
    mountings  all 18 in-envelope camera aims, no line touched

Stage 1 grids the three knobs that trade pace against margin. Stage 2 takes the
winner and walks the secondary knobs one at a time, keeping any change that is
faster and still passes everything.

    python optimize.py            both stages
    python optimize.py --stage2   refine from BASE below only
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

FIXED = ["-DRACE_BENCH_MODE=0", "-DSTEER_KFF=0.0f", "-DSPEED_MAX=100.0f"]

# Starting point for stage 2 if run on its own.
BASE = {"SPEED_MIN": "60.0f", "SPEED_SEVERITY_FULL": "1.00f", "STEER_PP_SCALE": "1.60f"}


def evaluate(cfg, tag="opt"):
    """Compile one configuration and score it on all three test sets."""
    exe = os.path.join(HERE, "opt_%s.exe" % tag)
    flags = ["-D%s=%s" % kv for kv in cfg.items()]
    cmd = [CC, "-O1", "-std=gnu99", "-o", exe] + SRC + \
          ["-I" + os.path.join(ROOT, "include")] + FIXED + flags + ["-lm"]
    if subprocess.run(cmd, capture_output=True, text=True).returncode != 0:
        return None

    def go(*args):
        return subprocess.run([exe] + list(args), capture_output=True, text=True).stdout

    out = go()
    laps = [float(x) for x in re.findall(r"laptime=\s*([0-9.]+)", out)][:5]
    exc = sum(int(x) for x in re.findall(r"excursions=(\d+)", out)[:5])
    clr = [float(x) for x in re.findall(r"minClearance=\s*([-+0-9.]+)", out)][:5]
    dnf = out.count("*DNF*")

    f = go("-fault")
    m = re.search(r"^clean run\s+\w+\s+(\d+)\s+([-+0-9.]+)", f, re.M)
    if m:
        exc += int(m.group(1))
        clr.append(float(m.group(2)))
    faults_clean = f.count("stayed inside the lines")

    e = go("-envelope")
    m = re.search(r"IN ENVELOPE\s*:\s*(\d+) configurations, \d+ finished, "
                  r"(\d+) with zero excursions, worst clearance ([-+0-9.]+)", e)
    env_n, env_clean, env_clr = (int(m.group(1)), int(m.group(2)), float(m.group(3))) \
        if m else (0, 0, -99.0)

    try:
        os.remove(exe)
    except OSError:
        pass

    if len(laps) < 5:
        return None

    ok = (exc == 0) and (dnf == 0) and (faults_clean == 8) and \
         (env_n == 18) and (env_clean == 18)
    return {"total": sum(laps), "exc": exc, "dnf": dnf, "laps": laps,
            "minclr": min(clr), "env_clean": env_clean, "env_clr": env_clr,
            "faults": faults_clean, "pass": ok}


def show(label, cfg, r):
    if r is None:
        print("  %-34s  build failed" % label)
        return
    print("  %-34s %8.2fs  clr%+6.2f  env %2d/18 (%+5.2f)  faults %d/8  %s"
          % (label, r["total"], r["minclr"], r["env_clean"], r["env_clr"],
             r["faults"], "PASS" if r["pass"] else "fail"))
    sys.stdout.flush()


def stage1():
    print("\n=== stage 1: the three knobs that trade pace against margin ===")
    grid = {
        "SPEED_MIN": ["60.0f", "68.0f", "76.0f", "84.0f"],
        "SPEED_SEVERITY_FULL": ["0.80f", "1.00f", "1.30f"],
        "STEER_PP_SCALE": ["1.30f", "1.60f", "2.00f"],
    }
    best = None
    for combo in itertools.product(*grid.values()):
        cfg = dict(zip(grid.keys(), combo))
        r = evaluate(cfg)
        label = " ".join("%s=%s" % (k.replace("SPEED_", "").replace("STEER_", ""), v)
                         for k, v in cfg.items())
        if r and r["pass"]:
            show(label, cfg, r)
            if (best is None) or (r["total"] < best[1]["total"]):
                best = (cfg, r)
    print()
    if best is None:
        print("  nothing passed all three test sets")
        return None
    print("  stage 1 winner: %.2fs  %s" % (best[1]["total"], best[0]))
    return best


def stage2(cfg, r):
    print("\n=== stage 2: secondary knobs, one at a time, keeping what helps ===")
    trials = [
        ("SPEED_ACC_MS2", ["4.50f", "6.00f"]),
        ("SPEED_ACC_EXIT_MS2", ["8.00f", "10.00f"]),
        ("SPEED_DEC_MS2", ["4.00f", "8.00f"]),
        ("STEER_KH", ["20.0f", "40.0f"]),
        ("STEER_KD", ["2.0f", "6.0f"]),
        ("SPEED_W_HEAD_FAR", ["0.80f", "1.30f"]),
        ("SPEED_W_CURV", ["0.60f", "1.10f"]),
        ("SPEED_SEE_FLOOR", ["0.35f", "0.60f"]),
        ("LINE_APEX_BIAS", ["0.75f", "0.95f"]),
        ("DIFF_GAIN", ["0.00f", "0.35f"]),
        ("SPEED_MIN", ["64.0f", "72.0f"]),
        ("STEER_PP_SCALE", ["1.45f", "1.75f"]),
    ]
    cur, curr = dict(cfg), r
    for key, values in trials:
        for v in values:
            trial = dict(cur)
            trial[key] = v
            rr = evaluate(trial)
            if rr and rr["pass"] and (rr["total"] < curr["total"] - 0.02):
                show("%s=%s  (kept, -%.2fs)" % (key, v, curr["total"] - rr["total"]),
                     trial, rr)
                cur, curr = trial, rr
            elif rr:
                show("%s=%s  (%s)" % (key, v, "slower" if rr["pass"] else "fails"),
                     trial, rr)
    return cur, curr


if __name__ == "__main__":
    if "--stage2" in sys.argv:
        r0 = evaluate(BASE)
        show("base", BASE, r0)
        cfg, r = stage2(BASE, r0)
    else:
        w = stage1()
        if w is None:
            sys.exit(1)
        cfg, r = stage2(w[0], w[1])

    print("\n" + "=" * 72)
    print("BEST that passes circuits + faults + all 18 in-envelope mountings")
    print("=" * 72)
    print("  %.2fs over the five circuits   (%s)"
          % (r["total"], "  ".join("%.2f" % x for x in r["laps"])))
    print("  worst clearance %+.2f cm, mountings %d/18, faults %d/8"
          % (r["minclr"], r["env_clean"], r["faults"]))
    print()
    for k, v in sorted(cfg.items()):
        print("  #define %-26s %s" % (k, v))
