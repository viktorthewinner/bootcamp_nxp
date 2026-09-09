#!/usr/bin/env python3
"""
Decode the car's flight recorder and report what actually happened.

The point of this is not to draw pretty graphs. It is to answer the questions that
cannot be answered from the code or from simulation, because they depend on the real
camera, the real mounting and the real track:

  - is the Pixy2 link healthy, and how fast is the control loop really running
  - how much of the time can the car see one line, both lines, or nothing
  - what corridor width does it actually measure, and is the camera aimed well
  - are the heading values in the range the corner thresholds assume
  - how much of the lap is spent at full speed, and what is throwing speed away
"""
import struct
import sys
import argparse

HDR = "<IHHIII8x"          # magic, version, recordSize, slots, count, uptimeMs, 8 pad
HDR_SIZE = struct.calcsize(HDR)
REC = "<HHhhhhhhhhhhBBBBHH"
REC_SIZE = struct.calcsize(REC)

F_HAVETRACK, F_BOTHEDGES, F_CHICANE = 0x01, 0x02, 0x04
F_CLAMPED, F_BRAKING, F_RUNNING, F_BENCH = 0x08, 0x10, 0x20, 0x40

FIELDS = ("frame dtUs steer speed targetX headNear headFar curv bias "
          "widthNear widthFar centerNear nValid nVectors laRow flags "
          "pixyErrors pixyTimeouts").split()


def pct(n, d):
    return (100.0 * n / d) if d else 0.0


def quantiles(vals, qs=(0.5, 0.9, 0.99)):
    if not vals:
        return [0.0] * len(qs)
    s = sorted(vals)
    return [s[min(len(s) - 1, int(q * len(s)))] for q in qs]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binfile")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    raw = open(args.binfile, "rb").read()
    if len(raw) < HDR_SIZE:
        print("File too short."); return 1

    magic, version, recsize, slots, count, uptime = struct.unpack_from(HDR, raw, 0)

    if magic != 0x314D4C54:
        print("Bad magic 0x%08X — expected 0x314D4C54 ('TLM1')." % magic)
        print("The buffer was never initialised. Either the firmware running on the")
        print("board is an older build without telemetry, or it never reached")
        print("Telemetry_Init(). Reflash and try again.")
        return 1
    if recsize != REC_SIZE:
        print("Record size mismatch: firmware says %d, decoder expects %d."
              % (recsize, REC_SIZE))
        return 1

    print("=" * 74)
    print("FLIGHT RECORDER  version %d   %d records written   uptime %.1f s"
          % (version, count, uptime / 1000.0))
    print("=" * 74)

    if count == 0:
        print("\nThe buffer is initialised but empty — not a single camera frame was")
        print("decoded. The Pixy2 never returned a good packet. Check that it is")
        print("powered, wired to the I2C pins, and running the line tracking program.")
        return 0

    n = min(count, slots)
    first = 0 if count <= slots else count % slots
    recs = []
    for i in range(n):
        off = HDR_SIZE + ((first + i) % slots) * REC_SIZE
        if off + REC_SIZE > len(raw):
            break
        recs.append(dict(zip(FIELDS, struct.unpack_from(REC, raw, off))))

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            f.write("frame,dt_us,steer,speed,targetX,headNear,headFar,curv,bias,"
                    "widthNear,widthFar,centerNear,nValid,nVectors,laRow,"
                    "haveTrack,bothEdges,chicane,clamped,braking,running,"
                    "pixyErrors,pixyTimeouts\n")
            for r in recs:
                fl = r["flags"]
                f.write("%d,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
                        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % (
                            r["frame"], r["dtUs"], r["steer"] / 10.0, r["speed"] / 10.0,
                            r["targetX"] / 10.0, r["headNear"] / 100.0,
                            r["headFar"] / 100.0, r["curv"] / 100.0, r["bias"] / 100.0,
                            r["widthNear"] / 10.0, r["widthFar"] / 10.0,
                            r["centerNear"] / 10.0, r["nValid"], r["nVectors"],
                            r["laRow"],
                            1 if fl & F_HAVETRACK else 0, 1 if fl & F_BOTHEDGES else 0,
                            1 if fl & F_CHICANE else 0, 1 if fl & F_CLAMPED else 0,
                            1 if fl & F_BRAKING else 0, 1 if fl & F_RUNNING else 0,
                            r["pixyErrors"], r["pixyTimeouts"]))
        print("\nWrote %d frames to %s" % (len(recs), args.csv))

    run = [r for r in recs if r["flags"] & F_RUNNING] or recs
    N = len(run)
    bench = bool(recs[-1]["flags"] & F_BENCH)
    if bench:
        print("\n*** BENCH MODE — the drive motors were held at zero. ***")

    # ---- camera link -----------------------------------------------------
    dts = [r["dtUs"] for r in run if 0 < r["dtUs"] < 65535]
    print("\n-- CAMERA LINK --")
    if dts:
        med, p90, p99 = quantiles(dts)
        print("  frame interval   median %5.1f ms   p90 %5.1f ms   p99 %5.1f ms"
              % (med / 1000.0, p90 / 1000.0, p99 / 1000.0))
        print("  frame rate       %.1f fps (median)" % (1e6 / med if med else 0))
        slow = sum(1 for d in dts if d > 2.5 * med)
        print("  stalls (>2.5x median): %d  (%.1f%% of frames)" % (slow, pct(slow, len(dts))))
    print("  pixy errors      %d" % run[-1]["pixyErrors"])
    print("  pixy timeouts    %d" % run[-1]["pixyTimeouts"])
    vecs = [r["nVectors"] for r in run]
    print("  vectors/frame    median %d   max %d   zero-vector frames %.1f%%"
          % (quantiles(vecs)[0], max(vecs), pct(sum(1 for v in vecs if v == 0), N)))

    # ---- what it could see ----------------------------------------------
    have = sum(1 for r in run if r["flags"] & F_HAVETRACK)
    both = sum(1 for r in run if r["flags"] & F_BOTHEDGES)
    print("\n-- WHAT THE CAR COULD SEE --")
    print("  usable track     %.1f%% of frames" % pct(have, N))
    print("  both black lines %.1f%%" % pct(both, N))
    print("  one line only    %.1f%%" % pct(have - both, N))
    print("  nothing at all   %.1f%%" % pct(N - have, N))
    nv = [r["nValid"] for r in run]
    print("  rows valid       median %d of 8   (how far ahead the model reached)"
          % quantiles(nv)[0])

    # ---- geometry --------------------------------------------------------
    wn = [r["widthNear"] / 10.0 for r in run if r["flags"] & F_HAVETRACK]
    cn = [r["centerNear"] / 10.0 for r in run if r["flags"] & F_HAVETRACK]
    print("\n-- CAMERA GEOMETRY (measured, not assumed) --")
    if wn:
        m = quantiles(wn)[0]
        print("  corridor width at the nearest row: median %.0f px" % m)
        print("  corridor centre at that row:       median %.0f px  (39 = straight ahead)"
              % quantiles(cn)[0])
        look = 68.0 / max(quantiles([r["widthFar"] / 10.0 for r in run
                                     if r["flags"] & F_HAVETRACK])[0], 1.0)
        print("  furthest useful look-ahead:        %.1f track widths" % look)

    # ---- headings --------------------------------------------------------
    hf = [abs(r["headFar"]) / 100.0 for r in run if r["flags"] & F_HAVETRACK]
    hn = [abs(r["headNear"]) / 100.0 for r in run if r["flags"] & F_HAVETRACK]
    print("\n-- HEADINGS (these set the corner thresholds) --")
    if hf:
        print("  |headNear|  median %.2f  p90 %.2f  p99 %.2f" % tuple(quantiles(hn)))
        print("  |headFar|   median %.2f  p90 %.2f  p99 %.2f" % tuple(quantiles(hf)))
        print("  CORNER_HEAD_IGNORE is 0.45 and CORNER_HEAD_FULL is 1.20:")
        print("    %.1f%% of frames read as a corner at all, %.1f%% as a full corner"
              % (pct(sum(1 for v in hf if v > 0.45), len(hf)),
                 pct(sum(1 for v in hf if v > 1.20), len(hf))))

    # ---- driving ---------------------------------------------------------
    sp = [r["speed"] / 10.0 for r in run]
    st = [abs(r["steer"]) / 10.0 for r in run]
    print("\n-- DRIVING --")
    print("  speed cmd        median %.0f  p90 %.0f  max %.0f" % (quantiles(sp)[0],
                                                                  quantiles(sp)[1], max(sp)))
    print("  |steer|          median %.0f  p90 %.0f  max %.0f" % (quantiles(st)[0],
                                                                  quantiles(st)[1], max(st)))
    sat = sum(1 for r in run if r["steer"] / 10.0 >= 44.0 or r["steer"] / 10.0 <= -59.0)
    print("  steering on the stop: %.1f%% of frames" % pct(sat, N))
    print("  chicane ignored:      %.1f%%" % pct(sum(1 for r in run if r["flags"] & F_CHICANE), N))
    print("  safety override:      %.1f%%" % pct(sum(1 for r in run if r["flags"] & F_CLAMPED), N))
    print("  braking:              %.1f%%" % pct(sum(1 for r in run if r["flags"] & F_BRAKING), N))

    # ---- verdicts --------------------------------------------------------
    print("\n-- WHAT THIS MEANS --")
    issues = []
    if run[-1]["pixyTimeouts"] > 0:
        issues.append("I2C is timing out (%d times). Check the Pixy2 wiring and ground."
                      % run[-1]["pixyTimeouts"])
    if dts:
        med = quantiles(dts)[0]
        if med > 25000:
            issues.append("Only %.0f fps. The control loop is starved; everything else "
                          "will look worse than it is." % (1e6 / med))
    if pct(have, N) < 90:
        issues.append("The track was unusable in %.0f%% of frames. Aim the camera before "
                      "touching any gain." % pct(N - have, N))
    if pct(both, N) < 40:
        issues.append("Both lines were visible in only %.0f%% of frames, so the car is "
                      "mostly inferring one edge. Raise the camera or tilt it up."
                      % pct(both, N))
    if wn and quantiles(cn)[0] < 33:
        issues.append("The corridor sits left of centre (median %.0f px, expected ~39). "
                      "The camera is aimed right of the car's axis, or CAM_CENTER_X "
                      "needs lowering." % quantiles(cn)[0])
    if wn and quantiles(cn)[0] > 45:
        issues.append("The corridor sits right of centre (median %.0f px, expected ~39). "
                      "The camera is aimed left of the car's axis, or CAM_CENTER_X "
                      "needs raising." % quantiles(cn)[0])
    if hf and quantiles(hf)[2] < 0.6:
        issues.append("|headFar| barely reaches %.2f, but CORNER_HEAD_FULL is 1.20 — the "
                      "car will never see a full corner and will not slow down enough. "
                      "Scale the CORNER_* thresholds to this run." % quantiles(hf)[2])
    if hf and quantiles(hf)[0] > 0.45:
        issues.append("Even the median |headFar| is %.2f, above CORNER_HEAD_IGNORE — the "
                      "car thinks it is permanently in a corner and will be slow "
                      "everywhere. Raise the CORNER_* thresholds." % quantiles(hf)[0])
    if pct(sat, N) > 15:
        issues.append("Steering is on the end stop %.0f%% of the time. It is running out "
                      "of lock — raise STEER_LIMIT_RIGHT, or slow down." % pct(sat, N))

    # Is the camera merging curves into single long vectors?
    corner = [r for r in run if abs(r["headFar"]) / 100.0 > 0.6]
    if corner:
        cv = [r["nVectors"] for r in corner]
        med_corner = quantiles(cv)[0]
        if med_corner <= 2:
            issues.append(
                "In corners the camera returns only %d vector(s) per frame. That means it "
                "is fitting ONE straight line across the whole curve. A straight line "
                "drawn across an arc touches it only at the ends and sits inside the bend "
                "in between, so the car reads the corner as starting earlier than it does "
                "and turns in too soon. Fix it in PixyMon's Expert tab: lower the line "
                "merging / minimum-line-length settings until a smooth corner shows 2-3 "
                "separate arrows instead of one." % med_corner)

    if issues:
        for i, s in enumerate(issues, 1):
            print("  %d. %s" % (i, s))
    else:
        print("  Nothing looks wrong. Link healthy, camera aimed sensibly, headings in")
        print("  range, steering not saturating.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
