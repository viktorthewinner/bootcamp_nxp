"""
steering.py - the lateral loop: what it is, why it behaves as it does at speed,
and what the firmware should do differently.

Run:  python steering.py

The plant, from the kinematic bicycle with a look-ahead measurement:

    e_dot   = v * psi                     lateral offset
    psi_dot = (v/L) * delta - kappa * v   heading error
    y_la    = e + d_la * psi              what the camera reports

              Y_la(s)      v   d_la * s + v
    P(s) =   --------- =  --- * ------------
             Delta(s)      L        s^2

A double integrator with one zero. Closed under a pure gain K (rad of steer per
metre of look-ahead error):

    omega_n = v * sqrt(K/L)          proportional to speed
    zeta    = (d_la/2) * sqrt(K/L)   independent of speed

so the loop does not destabilise with speed by itself. What destabilises it is
that crossover rises with speed while the camera and servo delay does not, so the
phase eaten by the delay grows as omega_c * T.

The firmware measures the error in PIXELS, not metres, and a pixel is worth
lat/d_la. That divides the effective metre-gain by the look-ahead distance, and
because LINE_LA_ROW slides outward with speed, the firmware already contains a
gain schedule. This file works out whether it is the right one.
"""

import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import plant as P

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

# --- camera geometry, matching race_config.h and track_sim.c -----------------
FOCAL_PX = 68.0
HALF_W_PX = 39.5  # 0.5 * PIXY_LINE_W
CAM_H = 0.18  # m
HORIZON_ROW = -4.0
ROW_Y = [50, 44, 38, 32, 26, 20, 13, 6]  # TRK_ROW_Y_INIT

STEER_KP = 78.0  # race_config.h
LA_ROW_MIN = 3
LA_ROW_MAX = 7


def hr(t):
    print("\n" + "=" * 74)
    print(t)
    print("=" * 74)


def row_distance(row_idx):
    """Ground distance to a track-model row, metres."""
    y = ROW_Y[row_idx]
    return (FOCAL_PX * CAM_H) / (y - HORIZON_ROW)


def gain_per_metre(kp, d_la):
    """Steering angle in rad produced per metre of look-ahead error.

    errN = f * lat / (d_la * halfW), steer = kp * errN, delta = steer/100 * lock.
    """
    return kp * (FOCAL_PX / HALF_W_PX) / d_la * (P.MAX_STEER_RAD / 100.0)


def loop_tf(w, v, d_la, kp, delay, servo_tau):
    """Open-loop frequency response of the lateral loop."""
    k = gain_per_metre(kp, d_la)
    s = 1j * w
    plant = (v / P.WHEELBASE) * ((d_la * s) + v) / (s ** 2)
    lag = np.exp(-s * delay) / ((servo_tau * s) + 1.0)
    return k * plant * lag


def margins(v, d_la, kp, delay, servo_tau):
    """Gain crossover, phase margin (deg)."""
    w = np.logspace(-2, 3, 20000)
    L = loop_tf(w, v, d_la, kp, delay, servo_tau)
    mag = np.abs(L)
    idx = np.nonzero(np.diff(np.sign(mag - 1.0)))[0]
    if len(idx) == 0:
        return float("nan"), float("nan")
    i = idx[0]
    wc = w[i]
    ph = np.degrees(np.angle(L[i]))
    return wc, 180.0 + ph


def la_distance_for_speed(frac):
    """What the firmware picks: LA row slides with speed fraction."""
    row = LA_ROW_MIN + ((LA_ROW_MAX - LA_ROW_MIN) * max(0.0, min(1.0, frac)))
    lo, hi = int(math.floor(row)), min(int(math.ceil(row)), LA_ROW_MAX)
    f = row - lo
    return ((1.0 - f) * row_distance(lo)) + (f * row_distance(hi))


# ---------------------------------------------------------------------------

def geometry():
    hr("1. what the camera rows actually are")
    print("  camera %.0f cm high, horizon at row %.0f, focal %.0f px\n"
          % (CAM_H * 100, HORIZON_ROW, FOCAL_PX))
    print("  %-8s %-10s %-14s %-12s" % ("row", "grid y", "distance", "gain K"))
    for i in range(len(ROW_Y)):
        d = row_distance(i)
        print("  %-8d %-10d %8.2f m     %8.3f rad/m" % (i, ROW_Y[i], d, gain_per_metre(STEER_KP, d)))
    ratio = row_distance(LA_ROW_MAX) / row_distance(LA_ROW_MIN)
    print("\n  LINE_LA_ROW_MIN=%d -> %.2f m, LINE_LA_ROW_MAX=%d -> %.2f m."
          % (LA_ROW_MIN, row_distance(LA_ROW_MIN), LA_ROW_MAX, row_distance(LA_ROW_MAX)))
    print("  Because the error is measured in pixels, sliding the row outward with")
    print("  speed divides the effective metre-gain by %.1f between crawling and flat" % ratio)
    print("  out. That is already a gain schedule - the question is its shape.")


def margin_vs_speed():
    hr("2. stability margins across the speed range")
    v_top = P.top_speed()
    speeds = np.linspace(0.3, v_top, 25)
    delay = P.SENSE_DELAY
    tau_s = P.SERVO_TAU

    schedules = {
        "fixed near row (d_la = %.2f m)" % row_distance(LA_ROW_MIN):
            lambda v: row_distance(LA_ROW_MIN),
        "fixed far row (d_la = %.2f m)" % row_distance(LA_ROW_MAX):
            lambda v: row_distance(LA_ROW_MAX),
        "firmware: row slides with speed":
            lambda v: la_distance_for_speed(v / v_top),
    }

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    print("  %-38s %8s %8s %8s %8s" % ("schedule", "v", "d_la", "wc", "PM"))
    for name, fn in schedules.items():
        wcs, pms, zetas = [], [], []
        for v in speeds:
            d = fn(v)
            wc, pm = margins(v, d, STEER_KP, delay, tau_s)
            k = gain_per_metre(STEER_KP, d)
            wcs.append(wc)
            pms.append(pm)
            zetas.append((d / 2.0) * math.sqrt(k / P.WHEELBASE))
        ax[0].plot(speeds, wcs, lw=1.8, label=name)
        ax[1].plot(speeds, pms, lw=1.8, label=name)
        ax[2].plot(speeds, zetas, lw=1.8, label=name)
        for v_show in (0.6, 1.2, v_top):
            j = int(np.argmin(np.abs(speeds - v_show)))
            print("  %-38s %6.2f  %6.2f m %7.2f %7.0f deg"
                  % (name if v_show == 0.6 else "", speeds[j], fn(speeds[j]),
                     wcs[j], pms[j]))
        print()

    ax[0].set_ylabel("crossover, rad/s")
    ax[1].set_ylabel("phase margin, deg")
    ax[1].axhline(45, color="k", ls=":", lw=1)
    ax[2].set_ylabel("damping zeta")
    for a in ax:
        a.set_xlabel("speed, m/s")
        a.grid(alpha=0.3)
        a.legend(fontsize=7)
    fig.suptitle("Lateral loop vs speed (sensor delay %.0f ms, servo tau %.0f ms)"
                 % (delay * 1e3, tau_s * 1e3))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "06_lateral_margins.png"), dpi=110)
    plt.close(fig)

    print("  Phase margin falls with speed in every schedule, because crossover rises")
    print("  with speed and the %.0f ms of camera-plus-servo delay does not move."
          % ((delay + tau_s) * 1e3))
    print("  Sliding the look-ahead row outward, which the firmware already does,")
    print("  is what keeps it in hand - it trades bandwidth for margin exactly where")
    print("  the margin is being eaten.")


def settling_distance():
    hr("3. how much track it takes to correct an error")
    v_top = P.top_speed()
    print("  A lateral loop is only useful if it settles in less track than there is")
    print("  before the next corner.\n")
    print("  %-10s %-10s %-10s %-12s %-12s" % ("speed", "d_la", "wc", "settle time", "settle dist"))
    for v in (0.6, 1.0, 1.4, 1.8, v_top):
        d = la_distance_for_speed(v / v_top)
        wc, pm = margins(v, d, STEER_KP, P.SENSE_DELAY, P.SERVO_TAU)
        ts = 4.0 / max(wc, 1e-6)
        print("  %6.2f m/s %7.2f m %7.2f rad/s %8.2f s %10.2f m" % (v, d, wc, ts, ts * v))
    print("\n  Settling distance is roughly constant - that is the double integrator's")
    print("  speed invariance showing up. It is also uncomfortably long compared with")
    print("  an NXP Cup straight, which is why the car cannot simply be given more speed.")


def pp_gain(d_la):
    """The geometrically correct metre-gain at this look-ahead: 2L/d_la^2."""
    return (2.0 * P.WHEELBASE) / (d_la * d_la)


def steady_state_corner():
    hr("4. where the car settles in a steady corner, and what decides it")

    print("  On a constant-radius corner the car needs a constant steering angle")
    print("  delta_ss = L/R, and a proportional law can only produce it from a")
    print("  standing look-ahead offset y_la = (L/R)/K.")
    print()
    print("  The tempting conclusion is that the car must therefore sit off the line")
    print("  by that much. It is wrong, and the reason matters: the aim point sits on")
    print("  the path, and a curved path is already displaced sideways within the")
    print("  look-ahead distance by its own sagitta, kappa*d_la^2/2. So")
    print()
    print("      y_la = e + d_la*psi + kappa*d_la^2 / 2")
    print()
    print("  and in steady state, with the heading settled, the real lateral error is")
    print()
    print("      e = kappa * ( L/K  -  d_la^2 / 2 )")
    print()
    print("  which is ZERO when K = 2L/d_la^2 - the pure pursuit gain. Above it the")
    print("  car tucks inside the line, below it runs wide. A curved path measured")
    print("  at a look-ahead point carries its own feedforward; the only question is")
    print("  whether the gain converts it at the right rate.\n")

    print("  %-8s %-10s %-14s %-14s %-10s %s"
          % ("LA row", "d_la", "firmware K", "geometric 2L/d^2", "ratio", "e per 1/m"))
    for i in range(LA_ROW_MIN, LA_ROW_MAX + 1):
        d = row_distance(i)
        k = gain_per_metre(STEER_KP, d)
        kg = pp_gain(d)
        e = (P.WHEELBASE / k) - (0.5 * d * d)
        print("  %-8d %7.2f m %10.3f     %10.3f      %7.2f   %+8.3f m"
              % (i, d, k, kg, k / kg, e))

    # Where the fixed gain is geometrically correct: 0.7031/d == 0.34/d^2
    k1 = gain_per_metre(STEER_KP, 1.0)  # = the 1/d coefficient
    d_cross = (2.0 * P.WHEELBASE) / k1
    print("\n  The two agree at d_la = %.2f m. Below that the fixed gain is too small" % d_cross)
    print("  and the car runs wide; above it the gain is too large and the car cuts.")
    print("  LINE_LA_ROW slides from %.2f m to %.2f m with speed, so the firmware"
          % (row_distance(LA_ROW_MIN), row_distance(LA_ROW_MAX)))
    print("  crosses that point every time it changes pace - it runs wide out of slow")
    print("  corners and cuts into fast ones, with one gain that cannot be right for")
    print("  both. This is what STEER_PP_SCALE replaces.")

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.4))
    dd = np.linspace(0.2, 1.4, 200)
    ax[0].plot(dd, [gain_per_metre(STEER_KP, d) for d in dd], lw=2, label="firmware, KP=78")
    ax[0].plot(dd, [pp_gain(d) for d in dd], lw=2, label="geometric 2L/d_la^2")
    ax[0].axvline(d_cross, color="k", ls=":", lw=1)
    ax[0].set_xlabel("look-ahead distance, m")
    ax[0].set_ylabel("gain, rad per metre")
    ax[0].set_title("one fixed gain cannot match a sliding look-ahead")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[0].set_ylim(0, 6)

    for R in (0.6, 0.9, 1.5):
        ax[1].plot(dd, [(1.0 / R) * ((P.WHEELBASE / gain_per_metre(STEER_KP, d))
                                     - (0.5 * d * d)) for d in dd],
                   lw=1.8, label="R = %.1f m" % R)
    ax[1].axhline(0, color="k", lw=1)
    ax[1].axvline(d_cross, color="k", ls=":", lw=1)
    ax[1].set_xlabel("look-ahead distance, m")
    ax[1].set_ylabel("steady lateral error, m")
    ax[1].set_title("+ is wide of the line, - is cutting")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "07_steady_state_corner.png"), dpi=110)
    plt.close(fig)
    return d_cross


def pure_pursuit_calibration():
    hr("5. the calibration-free form of the geometric law")

    print("  Pure pursuit needs the look-ahead DISTANCE, and the camera does not")
    print("  measure distance - it measures pixels, and what a pixel is worth depends")
    print("  entirely on how the camera happens to be bolted on.")
    print()
    print("  There is a way round it that the firmware already relies on elsewhere.")
    print("  Corridor width on screen is inversely proportional to distance, so with")
    print("  Dx the aim point's pixel offset and w the corridor width in pixels at")
    print("  that row:")
    print()
    print("      y_la = Dx * d_la / f          d_la = f * W / w")
    print()
    print("      delta = 2*L*y_la / d_la^2  =  2 * L * Dx * w / (f^2 * W)")
    print()
    print("  Camera height and tilt cancel completely. What is left is the Pixy2's")
    print("  own grid constant and L/W - wheelbase over track width, which is a tape")
    print("  measure and a division.\n")

    k = (200.0 * (P.WHEELBASE / 0.45)) / (FOCAL_PX * FOCAL_PX * P.MAX_STEER_RAD)
    print("      L/W        = %.3f" % (P.WHEELBASE / 0.45))
    print("      STEER_PP_K = 200*(L/W) / (f^2 * maxSteer) = %.6f" % k)
    print("      steer      = STEER_PP_K * Dx * w        steering units\n")

    print("  Cross-check against the metre form at each look-ahead row:")
    print("  %-8s %-9s %-9s %-16s %-16s" % ("row", "d_la", "w px", "geometric", "via Dx*w"))
    for i in range(LA_ROW_MIN, LA_ROW_MAX + 1):
        d = row_distance(i)
        w = FOCAL_PX * 0.45 / d
        lat = 0.10  # 10 cm of lateral offset at the aim point
        dx = FOCAL_PX * lat / d
        direct = pp_gain(d) * lat * 100.0 / P.MAX_STEER_RAD  # steering units
        viadxw = k * dx * w
        print("  %-8d %6.2f m %8.1f %12.2f u %13.2f u" % (i, d, w, direct, viadxw))
    print("\n  The two columns agree, which is the point: the firmware can compute the")
    print("  geometric steering angle from two numbers it already has, and stays")
    print("  correct if the camera is ever re-aimed.")


if __name__ == "__main__":
    geometry()
    margin_vs_speed()
    settling_distance()
    steady_state_corner()
    pure_pursuit_calibration()
    print("\nplots written to sim/out/")
