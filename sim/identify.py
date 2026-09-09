"""
identify.py - what the longitudinal plant actually is, and what closing the loop
around it would buy.

Run:  python identify.py
Writes plots into sim/out/ and prints the numbers the firmware needs.
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


def hr(title):
    print("\n" + "=" * 72)
    print(title)
    print("=" * 72)


# ---------------------------------------------------------------------------
# 1. Does the averaged model tell the truth?
# ---------------------------------------------------------------------------

def validate_averaged():
    hr("1. averaged model vs PWM-resolved model")
    dt = 1.0e-4
    T = 1.5
    n = int(T / dt)
    duty = 0.70

    fig, axes = plt.subplots(2, 2, figsize=(12, 7))
    rows = []

    for col, (decay, label) in enumerate([("slow", "slow decay (brake)"),
                                          ("fast", "fast decay (coast)")]):
        for pwm_hz, style, kstyle in [(P.PWM_HZ, "-", "k:"), (P.PWM_HZ_FAST, "--", "k-.")]:
            mp = P.MotorPWM(decay=decay, pwm_hz=pwm_hz)
            ma = P.MotorAvg(decay=decay, pwm_hz=pwm_hz)
            tv, vp, va, ripple = [], [], [], []
            for k in range(n):
                mp.step(duty, dt)
                ma.step(duty, dt)
                tv.append(k * dt)
                vp.append(mp.v)
                va.append(ma.v)
                ripple.append(mp.i_max - mp.i_min)

            axes[0][col].plot(tv, vp, style, lw=1.2,
                              label="PWM %0.0f kHz" % (pwm_hz / 1e3))
            axes[0][col].plot(tv, va, kstyle, lw=1.4,
                              label="averaged %0.0f kHz" % (pwm_hz / 1e3))
            axes[1][col].plot(tv, ripple, style, lw=1.2,
                              label="PWM %0.0f kHz" % (pwm_hz / 1e3))
            rows.append((label, pwm_hz, vp[-1], va[-1], ripple[-1]))

        axes[0][col].set_title(label)
        axes[0][col].set_ylabel("wheel speed, m/s")
        axes[0][col].legend(fontsize=8)
        axes[0][col].grid(alpha=0.3)
        axes[1][col].set_ylabel("current ripple, A")
        axes[1][col].set_xlabel("time, s")
        axes[1][col].legend(fontsize=8)
        axes[1][col].grid(alpha=0.3)

    fig.suptitle("Step to 70%% duty at %.1f V - averaged model vs PWM-resolved" % P.VBAT)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "01_model_validation.png"), dpi=110)
    plt.close(fig)

    print("  %-20s %8s %10s %10s %8s" % ("decay", "PWM", "v_pwm", "v_avg", "ripple"))
    for label, hz, vp, va, rip in rows:
        err = 100.0 * (va - vp) / max(vp, 1e-6)
        print("  %-20s %6.0f Hz %8.3f m/s %8.3f m/s %6.2f A   (avg model %+.1f%%)"
              % (label, hz, vp, va, rip, err))

    print("\n  The firmware runs 1 kHz today. tau_elec is %.2f ms, so at 1 kHz the"
          % (P.tau_elec() * 1e3))
    print("  winding current never settles inside a PWM period - that is the ripple")
    print("  column. It is wasted I^2R heating and it is audible. At 20 kHz the")
    print("  ripple all but disappears and the averaged model becomes exact.")


# ---------------------------------------------------------------------------
# 2. The steady-state duty -> speed map the firmware assumes is a straight line
# ---------------------------------------------------------------------------

def duty_speed_map():
    hr("2. steady-state duty -> speed")
    duties = np.linspace(0.0, 1.0, 41)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))

    results = {}
    for decay in ("slow", "fast"):
        vs = []
        for d in duties:
            m = P.MotorAvg(decay=decay)
            for _ in range(int(4.0 / 1e-3)):
                # constant load: rolling resistance referred to the armature
                f_roll = P.C_ROLL * P.MASS * P.G / 2.0
                t_load = f_roll * P.R_WHEEL / (P.GEAR_N * P.GEAR_EFF)
                m.step(d, 1e-3, load_torque=t_load)
            vs.append(m.v)
        results[decay] = np.array(vs)
        ax[0].plot(duties * 100.0, results[decay], lw=1.8, label=decay + " decay")

    ideal = duties * results["slow"][-1]
    ax[0].plot(duties * 100.0, ideal, "k:", lw=1.2, label="what the firmware assumes")
    ax[0].set_xlabel("duty, %")
    ax[0].set_ylabel("steady speed, m/s")
    ax[0].set_title("duty -> speed at %.1f V" % P.VBAT)
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3)

    for vb in (7.0, 8.0, 8.4, 12.0):
        vs = []
        for d in duties:
            m = P.MotorAvg(decay="slow", vbat=vb)
            for _ in range(int(4.0 / 1e-3)):
                f_roll = P.C_ROLL * P.MASS * P.G / 2.0
                t_load = f_roll * P.R_WHEEL / (P.GEAR_N * P.GEAR_EFF)
                m.step(d, 1e-3, load_torque=t_load)
            vs.append(m.v)
        ax[1].plot(duties * 100.0, vs, lw=1.6, label="%.1f V" % vb)
    ax[1].set_xlabel("duty, %")
    ax[1].set_ylabel("steady speed, m/s")
    ax[1].set_title("battery voltage moves the whole map")
    ax[1].legend(fontsize=9)
    ax[1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "02_duty_speed_map.png"), dpi=110)
    plt.close(fig)

    # Deadband and linearity error
    slow = results["slow"]
    fast = results["fast"]
    dead = duties[np.argmax(slow > 0.02)] * 100.0
    lin_err = 100.0 * np.max(np.abs(fast - ideal)) / max(ideal[-1], 1e-9)

    print("  duty at which the car first moves : %.0f %%   (gearbox stiction)" % dead)
    print("  worst deviation from the assumed straight line, fast decay: %.1f %% of full scale"
          % lin_err)
    print("  speed at 50%% duty  slow decay %.2f m/s   fast decay %.2f m/s"
          % (slow[20], fast[20]))
    print("\n  The firmware's DIR-pin wiring gives FAST decay forwards and SLOW decay")
    print("  in reverse, so the same number means two different speeds depending on")
    print("  sign, and neither is proportional to the command.")
    print("\n  Battery sensitivity: full duty gives %.2f m/s at 7.0 V and %.2f m/s at 8.4 V."
          % (P.top_speed(7.0), P.top_speed(8.4)))
    print("  That is a %.0f%% swing over one battery discharge, with nothing in the"
          % (100.0 * (P.top_speed(8.4) - P.top_speed(7.0)) / P.top_speed(8.4)))
    print("  firmware that notices.")
    return results


# ---------------------------------------------------------------------------
# 2b. Load regulation - how much speed a disturbance costs
# ---------------------------------------------------------------------------

def load_regulation():
    hr("2b. speed droop under load - the case for closing the loop")

    # Load expressed as an extra retarding force at the wheels, in newtons. A
    # corner adds tyre scrub, a ramp adds m*g*sin(theta), a dusty patch adds
    # rolling resistance. 1 N is about a 8.5% gradient for this car.
    forces = np.linspace(0.0, 2.5, 26)
    duty = 0.70

    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    droop = {}
    for decay, pwm_hz, label in [("slow", P.PWM_HZ, "slow decay, 1 kHz"),
                                 ("fast", P.PWM_HZ, "fast decay, 1 kHz (what the car does now)"),
                                 ("fast", P.PWM_HZ_FAST, "fast decay, 20 kHz"),
                                 ("slow", P.PWM_HZ_FAST, "slow decay, 20 kHz")]:
        vs = []
        for f in forces:
            m = P.MotorAvg(decay=decay, pwm_hz=pwm_hz)
            t_load = f * P.R_WHEEL / (2.0 * P.GEAR_N * P.GEAR_EFF)
            for _ in range(4000):
                m.step(duty, 1e-3, load_torque=t_load)
            vs.append(m.v)
        vs = np.array(vs)
        droop[label] = vs
        ax.plot(forces, vs, lw=1.8, label=label)

    ax.set_xlabel("extra retarding force at the wheels, N")
    ax.set_ylabel("steady speed, m/s")
    ax.set_title("Speed droop under load at %d%% duty" % int(duty * 100))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "02b_load_regulation.png"), dpi=110)
    plt.close(fig)

    print("  %-46s %8s %8s %10s" % ("configuration", "v @ 0 N", "v @ 1 N", "droop"))
    for label, vs in droop.items():
        i1 = int(np.argmin(np.abs(forces - 1.0)))
        d = 100.0 * (vs[0] - vs[i1]) / max(vs[0], 1e-9)
        print("  %-46s %6.2f    %6.2f    %6.1f %%" % (label, vs[0], vs[i1], d))

    print("\n  A 1 N disturbance is a 8.5%% gradient, or roughly what tyre scrub adds")
    print("  through a tight corner. The car loses a fifth of its speed to it and")
    print("  nothing in the firmware knows. That is the disturbance an integral term")
    print("  exists to reject - and it is the reason 'speed 75' is not a speed.")

    print("\n  WARNING about PWM frequency:")
    print("  At 1 kHz the forward direction is deeply DISCONTINUOUS, so fast decay")
    print("  happens to give almost the same DC gain as slow decay. Raise the PWM to")
    print("  20 kHz without changing the bridge wiring and conduction becomes")
    print("  continuous, the averaged winding voltage drops from D*V to (2D-1)*V, and")
    print("  the same duty gives %.2f m/s instead of %.2f m/s."
          % (droop["fast decay, 20 kHz"][0], droop["fast decay, 1 kHz (what the car does now)"][0]))
    print("  Raising the carrier frequency is only safe together with slow decay.")


# ---------------------------------------------------------------------------
# 3. Transfer function and frequency response
# ---------------------------------------------------------------------------

def freq_response():
    hr("3. transfer function, duty -> wheel speed")
    num2, den2 = P.tf_duty_to_speed()
    K, tau = P.tf_first_order()

    print("  second order:")
    print("      %.4e" % num2[0])
    print("    ------------------------------------------")
    print("    %.4e s^2 + %.4e s + %.4e" % (den2[0], den2[1], den2[2]))
    roots = np.roots(den2)
    print("    poles at %.1f and %.1f rad/s  (%.1f ms and %.1f ms)"
          % (roots[0].real, roots[1].real,
             -1e3 / roots[0].real, -1e3 / roots[1].real))
    print("\n  first-order reduction (this is what goes in the firmware):")
    print("                   %.3f" % K)
    print("    G(s) = -----------------      K = %.3f (m/s)/duty,  tau = %.0f ms"
          % (K, tau * 1e3))
    print("               %.3f s + 1" % tau)
    print("\n  The fast pole is %.0f x faster than the slow one, so dropping it costs"
          % (roots[0].real / roots[1].real))
    print("  nothing below a few hundred rad/s - far above anything the car does.")

    w = np.logspace(-1, 4, 500)
    s = 1j * w
    H2 = np.polyval(num2, s) / np.polyval(den2, s)
    H1 = K / ((tau * s) + 1.0)

    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].semilogx(w, 20 * np.log10(np.abs(H2)), lw=1.8, label="2nd order (full)")
    ax[0].semilogx(w, 20 * np.log10(np.abs(H1)), "--", lw=1.4, label="1st order")
    ax[0].set_ylabel("magnitude, dB")
    ax[0].legend(fontsize=9)
    ax[0].grid(which="both", alpha=0.3)
    ax[1].semilogx(w, np.degrees(np.angle(H2)), lw=1.8)
    ax[1].semilogx(w, np.degrees(np.angle(H1)), "--", lw=1.4)
    ax[1].set_ylabel("phase, deg")
    ax[1].set_xlabel("rad/s")
    ax[1].grid(which="both", alpha=0.3)
    fig.suptitle("duty -> wheel speed")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "03_bode_speed.png"), dpi=110)
    plt.close(fig)
    return K, tau


# ---------------------------------------------------------------------------
# 4. What closing the speed loop is worth
# ---------------------------------------------------------------------------

def _pi_gains(K, tau, omega_cl=12.0):
    """PI by pole cancellation: C(s) = Kp*(tau*s+1)/(tau*s) gives L(s)=Kp*K/(tau*s),
    a pure integrator crossing over at Kp*K/tau. Pick the crossover, get the gains."""
    kp = (tau * omega_cl) / K
    ki = kp / tau
    return kp, ki


def _sim_speed(mode, ref_fn, K, tau, kp, ki, vbat=P.VBAT, v0=1.0,
               dist_fn=None, T=2.5, dt=0.004, decay="slow"):
    """One run of the longitudinal loop against the real (nonlinear) motor."""
    m = P.MotorAvg(decay=decay, vbat=vbat)
    m.w = v0 * P.GEAR_N / P.R_WHEEL
    # Start the integrator where it would already be after cruising at v0, so the
    # test measures the response to the manoeuvre and not the loop waking up.
    integ = (v0 / K) / max(ki, 1e-9) if mode in ("pi", "pi+ff") else 0.0
    prev_ref = ref_fn(0.0)
    n = int(T / dt)
    ts = np.zeros(n)
    vs = np.zeros(n)
    ds = np.zeros(n)
    rs = np.zeros(n)

    for k in range(n):
        t = k * dt
        r = ref_fn(t)
        dref = (r - prev_ref) / dt
        prev_ref = r

        f_roll = P.C_ROLL * P.MASS * P.G / 2.0
        f_dist = (dist_fn(t) / 2.0) if dist_fn else 0.0
        t_load = (f_roll + f_dist) * P.R_WHEEL / (P.GEAR_N * P.GEAR_EFF)

        ff_static = r / K
        ff_dyn = (r + (tau * dref)) / K

        if mode == "open":
            duty = ff_static
        elif mode == "ff":
            duty = ff_dyn
        else:
            e = r - m.v
            base = ff_dyn if mode == "pi+ff" else 0.0
            duty = base + (kp * e) + (ki * integ)
            sat = max(-1.0, min(1.0, duty))
            # Conditional integration: keep integrating unless the output is
            # already against a stop AND the error is pushing it further in.
            # Back-calculating instead throws the accumulated value away every
            # time the loop leaves saturation, and the car then has to earn it
            # back at 1/Ki - which is exactly the slow tail out of every corner.
            if (duty == sat) or ((e * sat) < 0.0):
                integ += e * dt
            duty = sat

        duty = max(-1.0, min(1.0, duty))
        m.step(duty, dt, load_torque=t_load)
        ts[k], vs[k], ds[k], rs[k] = t, m.v, duty, r
    return ts, vs, ds, rs


def speed_controllers(K, tau):
    hr("4. open loop vs feedforward vs PI - closing the speed loop")

    kp, ki = _pi_gains(K, tau, omega_cl=12.0)
    print("  PI by pole cancellation, closed-loop crossover 12 rad/s (1.9 Hz):")
    print("      Kp = tau*w/K = %.3f      Ki = Kp/tau = %.3f" % (kp, ki))
    print("  The plant pole is at 1/tau = %.1f rad/s, so the loop is asked to be"
          % (1.0 / tau))
    print("  4x faster than the machine it is driving. Anything much beyond that and")
    print("  the 250 Hz control rate and the duty saturation start to bite.\n")

    modes = [("open", "open loop (today)", "-"),
             ("ff", "feedforward only", "--"),
             ("pi", "PI only", ":"),
             ("pi+ff", "PI + feedforward", "-.")]

    fig, ax = plt.subplots(2, 3, figsize=(15, 7))

    # ---- (a) step response: how fast can the car change speed at all ----
    # Kept well inside the envelope (top speed is %.2f m/s), otherwise the duty
    # saturates and every controller looks identical because the motor, not the
    # loop, is what is limiting.
    v_a, v_b = 0.60, 1.20
    step_ref = lambda t: v_a if t < 0.2 else v_b
    print("  (a) step %.2f -> %.2f m/s  (top speed is %.2f m/s, so there is headroom)"
          % (v_a, v_b, P.top_speed()))
    print("      %-22s %11s %10s %10s" % ("controller", "rise 10-90", "overshoot", "settle 2%"))
    for mode, lbl, style in modes:
        ts, vs, ds, rs = _sim_speed(mode, step_ref, K, tau, kp, ki, v0=v_a, T=2.0)
        seg = vs[ts >= 0.2]
        tt = ts[ts >= 0.2] - 0.2
        span = v_b - v_a
        lo, hi = v_a + (0.1 * span), v_a + (0.9 * span)
        if np.any(seg > hi):
            rise = tt[np.argmax(seg > hi)] - tt[np.argmax(seg > lo)]
            rise_s = "%8.0f ms" % (rise * 1e3)
        else:
            rise_s = "  never"  # steady-state offset: it never gets there
        over = 100.0 * (np.max(seg) - v_b) / span
        band = np.abs(seg - v_b) > 0.02 * v_b
        settle = tt[np.max(np.nonzero(band))] if np.any(band) else 0.0
        print("      %-22s %11s %9.1f %% %8.0f ms"
              % (lbl, rise_s, max(over, 0.0), settle * 1e3))
        ax[0][0].plot(ts, vs, style, lw=1.5, label=lbl)
        ax[1][0].plot(ts, ds * 100.0, style, lw=1.2, label=lbl)
    ax[0][0].plot(ts, rs, "k:", lw=1.4, label="reference")
    ax[0][0].set_title("(a) speed step", fontsize=10)

    # ---- (b) a feasible corner profile, battery healthy then sagged ----
    # Rate-limited so it is something the car could actually do: brake at the
    # deceleration the bridge can deliver, accelerate at what the motor gives.
    def corner_ref(t):
        if t < 0.30:
            return 1.90
        if t < 0.45:
            return 1.90 - (6.0 * (t - 0.30))
        if t < 1.05:
            return 1.00
        if t < 1.45:
            return 1.00 + (2.2 * (t - 1.05))
        return 1.88

    print("\n  (b) tracking a feasible corner profile")
    print("      %-22s %12s %12s" % ("controller", "RMS err 8.0V", "RMS err 7.0V"))
    for mode, lbl, style in modes:
        errs = []
        for vb in (8.0, 7.0):
            ts, vs, ds, rs = _sim_speed(mode, corner_ref, K, tau, kp, ki,
                                        vbat=vb, v0=1.90, T=2.0)
            errs.append(float(np.sqrt(np.mean((vs - rs) ** 2))))
            if vb == 8.0:
                ax[0][1].plot(ts, vs, style, lw=1.5, label=lbl)
            else:
                ax[1][1].plot(ts, vs, style, lw=1.5, label=lbl)
        print("      %-22s %10.3f   %10.3f  m/s" % (lbl, errs[0], errs[1]))
    ax[0][1].plot(ts, rs, "k:", lw=1.4, label="reference")
    ax[1][1].plot(ts, rs, "k:", lw=1.4)
    ax[0][1].set_title("(b) corner profile, 8.0 V", fontsize=10)
    ax[1][1].set_title("(b) same, battery sagged to 7.0 V", fontsize=10)

    # ---- (c) disturbance rejection ----
    dist = lambda t: 1.2 if 0.5 <= t < 1.5 else 0.0
    flat = lambda t: 1.60
    print("\n  (c) rejecting a 1.2 N load step (a ramp, or tyre scrub in a corner)")
    print("      %-22s %10s %10s" % ("controller", "max dip", "residual"))
    for mode, lbl, style in modes:
        ts, vs, ds, rs = _sim_speed(mode, flat, K, tau, kp, ki, v0=1.60,
                                    dist_fn=dist, T=2.2)
        win = (ts >= 0.5) & (ts < 1.5)
        dip = 1.60 - np.min(vs[win])
        resid = 1.60 - vs[(ts > 1.35) & (ts < 1.5)].mean()
        print("      %-22s %8.3f   %8.3f  m/s" % (lbl, dip, resid))
        ax[0][2].plot(ts, vs, style, lw=1.5, label=lbl)
        ax[1][2].plot(ts, ds * 100.0, style, lw=1.2)
    ax[0][2].plot(ts, rs, "k:", lw=1.4)
    ax[0][2].set_title("(c) 1.2 N load step", fontsize=10)

    for a in ax[0]:
        a.set_ylabel("speed, m/s")
        a.legend(fontsize=7)
        a.grid(alpha=0.3)
    for a in ax[1]:
        a.set_xlabel("time, s")
        a.grid(alpha=0.3)
    ax[1][0].set_ylabel("duty, %")
    ax[1][1].set_ylabel("speed, m/s")
    ax[1][2].set_ylabel("duty, %")
    ax[1][0].legend(fontsize=7)
    fig.suptitle("Longitudinal control: open loop vs feedforward vs PI")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "04_speed_loop.png"), dpi=110)
    plt.close(fig)

    print("\n  What the three columns say:")
    print("   (a) Open loop and feedforward never reach the target at all. Both compute")
    print("       duty = v/K from the no-load gain, and rolling resistance plus gearbox")
    print("       stiction then eat the difference. Adding the integrator is what")
    print("       removes the offset; adding feedforward on top halves the rise time")
    print("       again, because the loop no longer has to discover the operating point.")
    print("   (b) Feedforward alone still halves the tracking error - inverting a known")
    print("       348 ms pole is arithmetic and needs no sensor. But it degrades when")
    print("       the battery sags, because it is the gain K that has moved.")
    print("   (c) The decisive column. Feedforward cannot see a disturbance, so it eats")
    print("       the whole 0.5 m/s. This is what a wheel speed sensor actually buys,")
    print("       and nothing without one can substitute for it.")
    return kp, ki


# ---------------------------------------------------------------------------
# 5. Grip vs power - which one is really the limit
# ---------------------------------------------------------------------------

def grip_limits():
    hr("5. is this car grip limited or power limited?")

    print("  %-10s %-12s %-14s %-14s %s"
          % ("radius", "grip limit", "top speed 8.0V", "top speed 12V", "binding limit"))
    radii = [0.35, 0.5, 0.75, 0.9, 1.2, 1.5, 2.0, 3.0]
    v8 = P.top_speed(8.0)
    v12 = P.top_speed(12.0)
    for r in radii:
        vg = P.corner_speed_limit(r)
        lim8 = "grip" if vg < v8 else "POWER"
        lim12 = "grip" if vg < v12 else "POWER"
        print("  %6.2f m  %8.2f m/s  %10.2f (%-5s) %10.2f (%-5s)"
              % (r, vg, v8, lim8, v12, lim12))

    r8 = (v8 ** 2) / (P.MU * P.G)
    r12 = (v12 ** 2) / (P.MU * P.G)
    print("\n  Crossover radius: %.2f m at 8.0 V, %.2f m at 12 V." % (r8, r12))
    print("\n  This is the most consequential number in the whole analysis.")
    print("  An NXP Cup track's tightest corners are around 0.5-1.0 m radius. At 8 V")
    print("  the car cannot reach the grip limit in ANY of them - it is power limited")
    print("  everywhere, so every metre per second thrown away braking for a corner is")
    print("  simply lost, not traded for grip.")
    print("\n  And getting it back is slow: tau is %.0f ms, so recovering from a" % (P.tau_mech() * 1e3))
    print("  needless 0.5 m/s lift costs about %.0f ms of lap time even with the loop"
          % (1e3 * P.tau_mech() * math.log(1.0 / (1.0 - 0.5 / max(v8, 1e-6)))))
    print("  helping. That makes 'do not slow down unless you must' the dominant")
    print("  lap-time lever on this car, well ahead of cornering technique.")

    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    rr = np.linspace(0.3, 3.0, 300)
    ax.plot(rr, [P.corner_speed_limit(r) for r in rr], lw=2, label="grip limit sqrt(mu g R)")
    ax.axhline(v8, color="C1", ls="--", lw=1.5, label="power limit, 8.0 V")
    ax.axhline(v12, color="C3", ls="--", lw=1.5, label="power limit, 12 V")
    ax.fill_between(rr, 0, [min(P.corner_speed_limit(r), v8) for r in rr],
                    alpha=0.15, color="C2", label="achievable at 8.0 V")
    ax.axvspan(0.5, 1.0, alpha=0.10, color="C4", label="NXP Cup corner radii")
    ax.set_xlabel("corner radius, m")
    ax.set_ylabel("max speed, m/s")
    ax.set_title("Corner speed envelope")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "05_grip_envelope.png"), dpi=110)
    plt.close(fig)
    return r8


if __name__ == "__main__":
    print(P.summary())
    print("\n  NOTE: DRV8833 is rated 2.7-10.8 V. The 12 V the motors are labelled for")
    print("  is above its ceiling, so VBAT is modelled at %.1f V (2S LiPo)." % P.VBAT)
    validate_averaged()
    duty_speed_map()
    load_regulation()
    K, tau = freq_response()
    speed_controllers(K, tau)
    grip_limits()
    print("\nplots written to sim/out/")
