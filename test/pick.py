"""
pick.py - score candidate tunings on everything, not just lap time.

A configuration is only worth racing if it survives all three tests:

  circuits  five layouts, no line touched
  faults    a sixth layout plus dropped frames and blind patches
  mounting  the 18 camera mountings inside the aim envelope

The last one is the one a tuning search will quietly sacrifice if you let it.
Every mounting inside the envelope used to finish with zero line contact, and a
tune that gives that up has not made the car faster, it has made it dependent on
the camera bracket being exactly where it was when the tune was found.
"""

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

CANDIDATES = [
    ("new MIN76 PP1.3", ["-DSPEED_MAX=100.0f", "-DSPEED_MIN=76.0f", "-DSPEED_SEVERITY_FULL=1.30f", "-DSTEER_PP_SCALE=1.30f"]),
    ("new MIN68 PP1.6", ["-DSPEED_MAX=100.0f", "-DSPEED_MIN=68.0f", "-DSPEED_SEVERITY_FULL=1.00f", "-DSTEER_PP_SCALE=1.60f"]),
    ("new MIN68 PP2.0", ["-DSPEED_MAX=100.0f", "-DSPEED_MIN=68.0f", "-DSPEED_SEVERITY_FULL=1.00f", "-DSTEER_PP_SCALE=2.00f"]),
    ("new MIN64 PP1.6", ["-DSPEED_MAX=100.0f", "-DSPEED_MIN=64.0f", "-DSPEED_SEVERITY_FULL=1.00f", "-DSTEER_PP_SCALE=1.60f"]),
    ("new MIN64 PP2.0", ["-DSPEED_MAX=100.0f", "-DSPEED_MIN=64.0f", "-DSPEED_SEVERITY_FULL=1.00f", "-DSTEER_PP_SCALE=2.00f"]),
    ("new MIN60 PP1.6", ["-DSPEED_MAX=100.0f", "-DSPEED_MIN=60.0f", "-DSPEED_SEVERITY_FULL=1.00f", "-DSTEER_PP_SCALE=1.60f"]),
    ("new MIN60 PP2.0", ["-DSPEED_MAX=100.0f", "-DSPEED_MIN=60.0f", "-DSPEED_SEVERITY_FULL=1.00f", "-DSTEER_PP_SCALE=2.00f"]),
    ("old, orig tune", ["-DSPEED_CLOSED_LOOP=0", "-DSTEER_PURE_PURSUIT=0",
                        "-DSPEED_MAX=75.0f", "-DSPEED_MIN=44.0f", "-DSPEED_SEVERITY_FULL=1.00f"]),
    ("old, best tune", ["-DSPEED_CLOSED_LOOP=0", "-DSTEER_PURE_PURSUIT=0",
                        "-DSPEED_MAX=100.0f", "-DSPEED_MIN=60.0f", "-DSPEED_SEVERITY_FULL=0.80f"]),
]



def build(flags):
    exe = os.path.join(HERE, "pick.exe")
    cmd = [CC, "-O1", "-std=gnu99", "-o", exe] + SRC + \
          ["-I" + os.path.join(ROOT, "include"), "-DRACE_BENCH_MODE=0",
           "-DSTEER_KFF=0.0f"] + flags + ["-lm"]
    p = subprocess.run(cmd, capture_output=True, text=True)
    return exe if p.returncode == 0 else None


def score(exe):
    out = subprocess.run([exe], capture_output=True, text=True).stdout
    laps = [float(x) for x in re.findall(r"laptime=\s*([0-9.]+)", out)][:5]
    exc = sum(int(x) for x in re.findall(r"excursions=(\d+)", out)[:5])
    clr = [float(x) for x in re.findall(r"minClearance=\s*([-+0-9.]+)", out)][:5]

    f = subprocess.run([exe, "-fault"], capture_output=True, text=True).stdout
    m = re.search(r"^clean run\s+\w+\s+(\d+)\s+([-+0-9.]+)", f, re.M)
    if m:
        exc += int(m.group(1))
        clr.append(float(m.group(2)))
    fault_ok = f.count("stayed inside the lines")

    s = subprocess.run([exe, "-sweep"], capture_output=True, text=True).stdout
    m = re.search(r"IN ENVELOPE\s*:\s*(\d+) configurations, (\d+) finished, "
                  r"(\d+) with zero excursions, worst clearance ([-+0-9.]+)", s)
    env = (int(m.group(3)), int(m.group(1)), float(m.group(4))) if m else (0, 0, 0.0)

    return {"total": sum(laps), "exc": exc, "minclr": min(clr) if clr else 0.0,
            "env_clean": env[0], "env_n": env[1], "env_clr": env[2],
            "fault_ok": fault_ok}


if __name__ == "__main__":
    print("%-20s %9s %6s %8s %14s %10s" %
          ("tuning", "5 laps", "exc", "minClr", "in-envelope", "faults ok"))
    rows = []
    for name, flags in CANDIDATES:
        exe = build(flags)
        if exe is None:
            print("%-20s  build failed" % name)
            continue
        r = score(exe)
        rows.append((name, r))
        print("%-20s %8.2fs %6d %+8.2f %8d/%-5d %10d/8   worst mounting clr %+.2f"
              % (name, r["total"], r["exc"], r["minclr"],
                 r["env_clean"], r["env_n"], r["fault_ok"], r["env_clr"]))
        sys.stdout.flush()
    try:
        os.remove(os.path.join(HERE, "pick.exe"))
    except OSError:
        pass

    ok = [r for r in rows if (r[1]["exc"] == 0) and (r[1]["env_clean"] == r[1]["env_n"])]
    print()
    if ok:
        best = min(ok, key=lambda r: r[1]["total"])
        print("Fastest that is clean on the circuits AND on every in-envelope mounting:")
        print("   %s  at %.2fs" % (best[0].strip(), best[1]["total"]))
    else:
        print("No candidate was clean on both.")
