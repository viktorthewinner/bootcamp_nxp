"""
sweep.py - search the tuning space and report the fastest configuration that
never touches a line.

Comparing two control structures at the SAME gains is meaningless when one of
them changes what the gains mean. The open-loop firmware sends the planner's
number out as a duty, and the 348 ms motor lag then stops the car ever reaching
the speed the planner asked for - so its corner-speed floor is not really
SPEED_MIN at all. Close the loop and the car obeys, which looks like a
regression until the planner is retuned for a machine that now does as it is
told.

So each structure is given its own search, and the comparison is between the
best each can do while staying inside the black lines.

    python sweep.py            the default comparison
    python sweep.py --battery  how each one holds up as the battery drains
"""

import itertools
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CC = os.environ.get("CC", "gcc")
# A run that clears the black line by 4 mm is luck, not a setup. The headline
# number is still the fastest clean lap, but this is the one worth racing.
SAFE_CLR = 1.5

# The winners of the search below, each at its own best tune.
BEST_OL = ["-DSPEED_CLOSED_LOOP=0", "-DSTEER_KFF=0.0f", "-DSTEER_PURE_PURSUIT=0",
           "-DSPEED_MAX=100.0f", "-DSPEED_MIN=60.0f", "-DSPEED_SEVERITY_FULL=0.80f"]
BEST_CL = ["-DSTEER_KFF=0.0f", "-DSPEED_MAX=100.0f", "-DSPEED_MIN=68.0f",
           "-DSPEED_SEVERITY_FULL=1.00f", "-DSTEER_PP_SCALE=1.30f"]
BEST_CL_FAST = ["-DSTEER_KFF=0.0f", "-DSPEED_MAX=100.0f", "-DSPEED_MIN=68.0f",
                "-DSPEED_SEVERITY_FULL=1.30f", "-DSTEER_PP_SCALE=1.60f"]

SRC = [os.path.join(HERE, "track_sim.c"),
       os.path.join(ROOT, "source", "track.c"),
       os.path.join(ROOT, "source", "racing_line.c"),
       os.path.join(ROOT, "source", "intersection.c"),
       os.path.join(ROOT, "source", "driver.c"),
       os.path.join(ROOT, "source", "speed_ctl.c")]


def run(flags, tag):
    exe = os.path.join(HERE, "sweep_%s.exe" % tag)
    cmd = [CC, "-O1", "-std=gnu99", "-o", exe] + SRC + \
          ["-I" + os.path.join(ROOT, "include"), "-DRACE_BENCH_MODE=0"] + flags + ["-lm"]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        return None, p.stderr.strip().splitlines()[-3:]
    p = subprocess.run([exe], capture_output=True, text=True)
    out = p.stdout
    laps = [float(x) for x in re.findall(r"laptime=\s*([0-9.]+)", out)][:5]
    exc = sum(int(x) for x in re.findall(r"excursions=(\d+)", out)[:5])
    clr = [float(x) for x in re.findall(r"minClearance=\s*([-+0-9.]+)", out)][:5]
    dnf = out.count("*DNF*")

    # The five headline circuits are not the whole test set. The fault suite runs
    # a sixth layout, and its "clean run" row is a lap with no faults injected at
    # all - so a configuration that crosses a line there has crossed a line, and
    # scoring only the first five quietly hides it.
    p2 = subprocess.run([exe, "-fault"], capture_output=True, text=True)
    m = re.search(r"^clean run\s+(\w+)\s+(\d+)\s+([-+0-9.]+)", p2.stdout, re.M)
    if m:
        exc += int(m.group(2))
        clr.append(float(m.group(3)))

    try:
        os.remove(exe)
    except OSError:
        pass
    if len(laps) < 5:
        return None, ["only %d circuits finished" % len(laps)]
    return {"total": sum(laps), "exc": exc, "dnf": dnf,
            "minclr": min(clr) if clr else 0.0, "laps": laps}, None


def search(name, base_flags, grid, tag):
    print("\n=== %s ===" % name)
    best = None
    safe = None
    tried = 0
    for combo in itertools.product(*grid.values()):
        flags = list(base_flags)
        for k, v in zip(grid.keys(), combo):
            flags.append("-D%s=%s" % (k, v))
        r, err = run(flags, tag)
        tried += 1
        if r is None:
            continue
        clean = (r["exc"] == 0) and (r["dnf"] == 0)
        if clean and ((best is None) or (r["total"] < best[0]["total"])):
            best = (r, dict(zip(grid.keys(), combo)))
        if clean and (r["minclr"] >= SAFE_CLR) and            ((safe is None) or (r["total"] < safe[0]["total"])):
            safe = (r, dict(zip(grid.keys(), combo)))
    if best is None:
        print("  no clean configuration found in %d tried" % tried)
        return None
    r, cfg = best
    print("  fastest clean of %d tried: %.2fs   minClearance %+.2f cm"
          % (tried, r["total"], r["minclr"]))
    print("      " + "  ".join("%s=%s" % kv for kv in cfg.items()))
    print("      per circuit: " + "  ".join("%.2f" % x for x in r["laps"]))
    if safe is None:
        print("  nothing kept %.1f cm of margin anywhere in the grid" % SAFE_CLR)
    else:
        rs, cs = safe
        print("  fastest with >= %.1f cm margin: %.2fs   minClearance %+.2f cm"
              % (SAFE_CLR, rs["total"], rs["minclr"]))
        print("      " + "  ".join("%s=%s" % kv for kv in cs.items()))
    return base_flags + ["-D%s=%s" % kv for kv in cfg.items()], r


def battery_check(configs):
    print("\n=== robustness: the same tuning as the battery drains ===")
    print("  The planner's numbers are calibrated against SPEED_TOP_MS. Open loop")
    print("  sends them out as a duty, so when the pack sags every one of them")
    print("  quietly means something slower. Closed loop re-derives the duty from")
    print("  the map each step, so the only thing that moves is the ceiling.\n")
    print("  %-22s %8s %8s %8s %8s" % ("configuration", "7.0 V", "7.4 V", "8.0 V", "8.4 V"))
    for name, flags in configs:
        row = []
        for vb in (7.0, 7.4, 8.0, 8.4):
            top = {7.0: 1.73, 7.4: 1.84, 8.0: 2.01, 8.4: 2.12}[vb]
            f = list(flags) + ["-DSIM_VBAT=%.1f" % vb, "-DSPEED_TOP_MS=%.2ff" % top]
            r, err = run(f, "bat")
            if r is None:
                row.append("build?")
            elif r["dnf"] or r["exc"]:
                row.append("%.1f!%d" % (r["total"], r["exc"]))
            else:
                row.append("%.2f" % r["total"])
        print("  %-22s %8s %8s %8s %8s" % (name, *row))
    print("\n  A bare number is a clean run. 'x!n' means it finished in x seconds")
    print("  having crossed a line n times, which in a race is a disqualification.")


if __name__ == "__main__":
    if "--push" in sys.argv:
        # Outright pace. The car is power limited at every corner radius an NXP
        # Cup layout contains (sim/identify.py section 5), so the fastest lap is
        # the one that stops slowing down for corners it never needed to slow
        # for. The only hard constraint left is that a lap touching a line does
        # not count.
        search("maximum attack",
               ["-DSTEER_KFF=0.0f", "-DSPEED_MAX=100.0f"],
               {"SPEED_MIN": ["68.0f", "76.0f", "84.0f", "92.0f", "100.0f"],
                "SPEED_SEVERITY_FULL": ["1.00f", "1.40f", "1.90f"],
                "STEER_PP_SCALE": ["1.30f", "1.60f", "2.00f", "2.40f"]},
               "push")
        sys.exit(0)

    if "--final" in sys.argv:
        # Same grid, same six-layout scoring, for each control structure.
        g = {"SPEED_MAX": ["85.0f", "100.0f"],
             "SPEED_MIN": ["52.0f", "60.0f", "68.0f", "76.0f", "84.0f", "92.0f"],
             "SPEED_SEVERITY_FULL": ["0.80f", "1.00f", "1.30f"]}
        search("A. today: open loop speed, fixed-gain steering",
               ["-DSPEED_CLOSED_LOOP=0", "-DSTEER_KFF=0.0f",
                "-DSTEER_PURE_PURSUIT=0"], g, "fa")
        g2 = dict(g)
        g2["STEER_PP_SCALE"] = ["1.30f", "1.60f", "2.00f"]
        search("C. closed loop speed + geometric steering",
               ["-DSTEER_KFF=0.0f"], g2, "fc")
        sys.exit(0)

    if "--push3" in sys.argv:
        # The tight circuit is steering-LOCK limited, not grip or power limited:
        # the command sits on STEER_LIMIT_LEFT/RIGHT through the 55 cm hairpin,
        # and a saturated servo is an open loop. So the cue to slow on is not
        # "there is a corner" but "the car is running out of lock", which is what
        # SPEED_W_STEER already measures. Raising it lets the car stay fast
        # everywhere else and lift only where it is actually out of steering.
        search("attack, slowing only for steering saturation",
               ["-DSTEER_KFF=0.0f", "-DSPEED_MAX=100.0f",
                "-DSPEED_SEVERITY_FULL=1.00f"],
               {"SPEED_MIN": ["76.0f", "84.0f", "88.0f", "92.0f"],
                "SPEED_W_STEER": ["0.90f", "1.20f", "1.60f", "2.00f"],
                "STEER_PP_SCALE": ["1.30f", "1.60f", "2.00f"]},
               "push3")
        sys.exit(0)

    if "--push2" in sys.argv:
        # Round two: having established that the car barely needs to lift, the
        # remaining corner speed is in how much of the track the racing line is
        # allowed to use. SAFE_MARGIN_FRAC is the keep-out band beside each black
        # line and LINE_APEX_BIAS is how hard the car dives for the apex - both
        # straighten the path, which is what raises corner speed on a car that is
        # not grip limited.
        search("maximum attack, racing line opened up",
               ["-DSTEER_KFF=0.0f", "-DSPEED_MAX=100.0f",
                "-DSPEED_SEVERITY_FULL=1.00f"],
               {"SPEED_MIN": ["88.0f", "92.0f", "96.0f", "100.0f"],
                "STEER_PP_SCALE": ["1.60f", "2.00f"],
                "SAFE_MARGIN_FRAC": ["0.16f", "0.19f", "0.22f"],
                "LINE_APEX_BIAS": ["0.85f", "0.95f", "1.00f"]},
               "push2")
        sys.exit(0)

    if "--battery" in sys.argv:
        battery_check([("A today, best tune", BEST_OL),
                       ("C closed+geometric", BEST_CL),
                       ("C same, PP_SCALE 1.6", BEST_CL_FAST)])
        sys.exit(0)

    grid = {
        "SPEED_MAX": ["85.0f", "100.0f"],
        "SPEED_MIN": ["44.0f", "52.0f", "60.0f", "68.0f"],
        "SPEED_SEVERITY_FULL": ["0.80f", "1.00f", "1.30f"],
    }

    search("A. today: open loop speed, fixed-gain steering",
           ["-DSPEED_CLOSED_LOOP=0", "-DSTEER_KFF=0.0f", "-DSTEER_PURE_PURSUIT=0"],
           grid, "a")

    search("B. closed loop speed, fixed-gain steering",
           ["-DSTEER_KFF=0.0f", "-DSTEER_PURE_PURSUIT=0"], grid, "b")

    g2 = dict(grid)
    g2["STEER_PP_SCALE"] = ["1.30f", "1.60f", "2.00f"]
    search("C. closed loop speed, geometric (pure pursuit) steering",
           ["-DSTEER_KFF=0.0f"], g2, "c")
