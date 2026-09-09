"""
plant.py - physical model of the NXP Cup car.

Everything the control design rests on lives here: the 25GA-370 gearmotors, the
DRV8833 bridge that drives them, the drivetrain, and the vehicle.

Two models of the same hardware:

  MotorPWM   - PWM-resolved. Integrates the winding current every few microseconds,
               so it reproduces the decay mode (coast vs brake during the PWM
               off-time), discontinuous conduction, and the current limit. Slow,
               but it is the ground truth the averaged model is checked against.

  MotorAvg   - the averaged two-state model the transfer functions come from.
               Fast enough to run a whole lap.

Parameters are derived from the published 25GA-370 numbers rather than invented:
no-load speed, no-load current and stall current pin down Ke, Kt, R and b, and
everything else follows. Where a number is genuinely uncertain it is marked, and
sweeps in analyze.py show how much it matters.
"""

import math

# ----------------------------------------------------------------------------
# Motor: 25GA-370, 12 V, 1000 rpm at the output shaft
# ----------------------------------------------------------------------------
# A 370-size armature free-runs at about 9600 rpm on 12 V, so 1000 rpm out is the
# standard 1:9.6 gearset. That fixes the ratio without having to guess it.

GEAR_N = 9.6  # gearbox reduction, motor rev per output rev
GEAR_EFF = 0.70  # gearbox efficiency, spur/planetary at this size

V_RATED = 12.0  # V, the voltage the 1000 rpm figure is quoted at
W_OUT_NL = 1000.0 * 2.0 * math.pi / 60.0  # rad/s, output shaft, no load
W_MOT_NL = W_OUT_NL * GEAR_N  # rad/s, armature, no load
I_NL = 0.12  # A, no-load current (datasheet band 0.10-0.15)
I_STALL = 3.0  # A, stall current at 12 V (datasheet band 2.5-3.2)

R_A = V_RATED / I_STALL  # armature + brush resistance, ohm
L_A = 2.0e-3  # armature inductance, H. 370-size motors sit at 1-3 mH.

# Back-EMF constant from the no-load operating point: at no load the applied
# voltage is spent on I_NL*R plus back-EMF, so Ke falls straight out.
KE = (V_RATED - (I_NL * R_A)) / W_MOT_NL  # V*s/rad
KT = KE  # N*m/A - the same number in SI, always

# Viscous friction, again from the no-load point: everything the no-load current
# produces is spent turning the motor against its own drag.
B_M = (KT * I_NL) / W_MOT_NL  # N*m*s/rad

J_MOT = 1.2e-6  # kg*m^2, 370 armature. Uncertain +/- 50%, swept in analyze.py.

# Coulomb friction referred to the armature: gearboxes have a breakaway torque
# that a purely viscous model misses, and it is what creates the low-duty
# deadband the car actually feels when creeping.
T_COUL = 1.6e-3  # N*m at the armature (~1.5% of stall torque)

# ----------------------------------------------------------------------------
# DRV8833 dual H-bridge
# ----------------------------------------------------------------------------
# TI DRV8833: 2.7-10.8 V operating, 1.5 A RMS / 2 A peak per bridge, internal
# current limit. The 10.8 V ceiling is why V_BAT below is not 12 V - see the
# note printed by analyze.py.

VBAT = 8.0  # V, 2S LiPo mid-discharge. The single most influential parameter.
R_DS_ON = 0.36  # ohm, high-side + low-side on-resistance in series with the motor
V_DIODE = 0.9  # V, body-diode drop during fast decay
I_LIMIT = 2.0  # A, bridge peak current limit

PWM_HZ = 1000.0  # what the firmware runs today (CTIMER0: 2343750 / 2343)
PWM_HZ_FAST = 20000.0  # the proposal

# ----------------------------------------------------------------------------
# Drivetrain and vehicle
# ----------------------------------------------------------------------------

R_WHEEL = 0.032  # m, 64 mm diameter
MASS = 1.20  # kg, car with battery
WHEELBASE = 0.17  # m, matches track_sim.c
TRACK_W = 0.15  # m, rear axle track width
J_WHEEL = 1.5e-5  # kg*m^2 per wheel

MU = 1.05  # tyre-road friction (track_sim.c already assumes 1.05 g)
G = 9.81

C_ROLL = 0.015  # rolling resistance coefficient
C_DRAG = 0.0035  # N/(m/s)^2 - aero, negligible at 2 m/s but kept for honesty

MAX_STEER_RAD = 30.0 * math.pi / 180.0  # servo lock, matches track_sim.c
SERVO_TAU = 0.060  # s, first-order lag of a hobby servo under load
SERVO_RATE = 6.0  # rad/s, slew limit (~0.17 s per 60 deg)

# Latency from photons to a new servo command: Pixy2 exposure and line extraction,
# the I2C read, and one control step. Measured on similar setups at 25-40 ms.
SENSE_DELAY = 0.030  # s


# ----------------------------------------------------------------------------
# Derived quantities used all over the place
# ----------------------------------------------------------------------------

def inertia_at_armature(mass_share=0.5):
    """Total inertia seen by one armature, kg*m^2.

    The car's translational mass is what dominates: it appears at the armature
    divided by the square of the total reduction, but that reduction is large
    enough that it still outweighs the rotor.
    """
    m = MASS * mass_share
    j_trans = (m * R_WHEEL ** 2) / (GEAR_N ** 2 * GEAR_EFF)
    j_wheel = J_WHEEL / (GEAR_N ** 2 * GEAR_EFF)
    return J_MOT + j_wheel + j_trans


def tau_mech(mass_share=0.5):
    """Dominant (mechanical) time constant of duty -> speed, seconds."""
    j = inertia_at_armature(mass_share)
    return (j * (R_A + R_DS_ON)) / ((R_A + R_DS_ON) * B_M + KT * KE)


def tau_elec():
    """Electrical time constant, seconds."""
    return L_A / (R_A + R_DS_ON)


def dc_gain_duty_to_speed(vbat=None, mass_share=0.5):
    """Steady-state m/s per unit of duty (duty in 0..1), no load torque."""
    vb = VBAT if vbat is None else vbat
    r = R_A + R_DS_ON
    w_mot = (vb * KT) / (r * B_M + KT * KE)
    return w_mot * R_WHEEL / GEAR_N


def top_speed(vbat=None):
    """Steady-state speed at full duty including rolling resistance, m/s."""
    vb = VBAT if vbat is None else vbat
    lo, hi = 0.0, 10.0
    for _ in range(200):
        v = 0.5 * (lo + hi)
        if accel_at(v, 1.0, vbat=vb) > 0.0:
            lo = v
        else:
            hi = v
    return 0.5 * (lo + hi)


def accel_at(v, duty, vbat=None):
    """Steady-state longitudinal acceleration at speed v and duty, m/s^2.

    Both motors, averaged model, current in equilibrium. Used for the friction
    circle and for the top-speed solve above.
    """
    vb = VBAT if vbat is None else vbat
    r = R_A + R_DS_ON
    w_mot = v * GEAR_N / R_WHEEL
    i = ((duty * vb) - (KE * w_mot)) / r
    i = max(-I_LIMIT, min(I_LIMIT, i))
    t_arm = (KT * i) - (B_M * w_mot) - (T_COUL if w_mot > 1.0 else 0.0)
    f_drive = 2.0 * t_arm * GEAR_N * GEAR_EFF / R_WHEEL
    f_roll = C_ROLL * MASS * G * (1.0 if v > 0.01 else 0.0)
    f_drag = C_DRAG * v * v
    return (f_drive - f_roll - f_drag) / MASS


# ----------------------------------------------------------------------------
# Averaged motor model - the one the transfer functions describe
# ----------------------------------------------------------------------------

class MotorAvg:
    """Two-state averaged DC motor: winding current and armature speed.

    `duty` is signed, -1..+1. `decay` selects what the bridge does during the
    PWM off-time:

      'slow' - both outputs low, the winding is shorted. The averaged terminal
               voltage is duty * VBAT and regulation is stiff.
      'fast' - both outputs high-Z. While current still flows it is pushed back
               into the supply through the body diodes, so the winding sees
               -VBAT, and the averaged voltage is (2*duty - 1) * VBAT. Once the
               current reaches zero the terminals are open and the winding sees
               nothing at all.

    The distinction is not academic: the firmware's bridge wiring gives fast
    decay forwards and slow decay in reverse, which is the asymmetry analyze.py
    measures.
    """

    def __init__(self, vbat=None, mass_share=0.5, decay='slow', pwm_hz=PWM_HZ):
        self.vbat = VBAT if vbat is None else vbat
        self.j = inertia_at_armature(mass_share)
        self.mass_share = mass_share
        self.decay = decay
        self.pwm_hz = pwm_hz
        self.i = 0.0
        self.w = 0.0  # armature speed, rad/s

    @property
    def v(self):
        """Wheel speed in m/s."""
        return self.w * R_WHEEL / GEAR_N

    def avg_current(self, duty):
        """Average winding current over one PWM period, exactly.

        The electrical time constant is 700x shorter than the mechanical one, so
        within a PWM period the speed is constant and the current waveform can be
        solved in closed form instead of integrated.

        Over the on-time the current heads for I_on = (V - E)/R; over the off-time
        it heads for I_off, which is -E/R when the bridge brakes and
        -(V + 2*Vd + E)/R when it coasts into the body diodes. Integrating both
        exponentials and using i(t_z) = 0 at the zero crossing, almost everything
        cancels and the average is just

            <i> = ( I_on * t_on  +  I_off * min(t_z, t_off) ) / T

        which is exact in continuous conduction (t_z >= t_off) and in
        discontinuous conduction (t_z < t_off, the winding open for the rest of
        the period) with no case analysis beyond that one min().
        """
        d = min(1.0, abs(duty))
        sign = 1.0 if duty >= 0.0 else -1.0
        v_bat = self.vbat
        r = R_A + R_DS_ON
        tau = L_A / r
        period = 1.0 / self.pwm_hz
        t_on = d * period
        t_off = period - t_on

        # Back-EMF as seen in the direction the bridge is driving.
        emf = KE * self.w * sign

        i_on = (v_bat - emf) / r
        if self.decay == 'slow':
            i_off = -emf / r
        else:
            i_off = (-(v_bat + (2.0 * V_DIODE)) - emf) / r

        i_on = max(-I_LIMIT, min(I_LIMIT, i_on))

        if self.decay == 'slow':
            # Both low-side FETs are on, so the winding is shorted and the current
            # is free to flow either way. It never becomes discontinuous, and the
            # average collapses to the textbook (D*V - E)/R.
            t_z = t_off
        else:
            # Coasting into the body diodes. They block reverse current, so the
            # winding CAN go open part way through the off-time. Decide which
            # regime this is from the steady-state periodic minimum: solve the
            # two exponential segments for i_min and see whether it stays above
            # zero.
            a = math.exp(-t_on / tau)
            b = math.exp(-t_off / tau)
            i_min = ((i_off * (1.0 - b)) + (i_on * b * (1.0 - a))) / max(1.0 - (a * b), 1e-12)

            if i_min > 0.0:
                t_z = t_off  # continuous conduction after all
            else:
                # Discontinuous: the current starts each period at zero.
                i_peak = i_on * (1.0 - a)
                i_peak = max(-I_LIMIT, min(I_LIMIT, i_peak))
                if abs(i_peak) < 1e-12:
                    # Nothing was ever conducting, so nothing decays either. The
                    # winding is open for the whole period.
                    t_z = 0.0
                elif (i_peak > 0.0) and (i_off < 0.0):
                    t_z = tau * math.log(1.0 + (i_peak / -i_off))
                elif (i_peak < 0.0) and (i_off > 0.0):
                    t_z = tau * math.log(1.0 + (-i_peak / i_off))
                else:
                    t_z = t_off
                t_z = max(0.0, min(t_z, t_off))

        i_avg = ((i_on * t_on) + (i_off * t_z)) / period
        i_avg = max(-I_LIMIT, min(I_LIMIT, i_avg))
        return sign * i_avg

    def step(self, duty, dt, load_torque=0.0):
        """Advance by dt with a signed duty and an external load at the armature."""
        duty = max(-1.0, min(1.0, duty))
        self.i = self.avg_current(duty)

        t = (KT * self.i) - (B_M * self.w) - load_torque
        if abs(self.w) > 1.0:
            t -= math.copysign(T_COUL, self.w)
        elif abs(t) < T_COUL:
            t = 0.0
        self.w += (t / self.j) * dt
        return self.v


# ----------------------------------------------------------------------------
# PWM-resolved motor model - ground truth
# ----------------------------------------------------------------------------

class MotorPWM:
    """Same motor, but the PWM carrier is integrated instead of averaged.

    Used only to check that MotorAvg tells the truth, and to show what raising
    the PWM frequency does to the current ripple.
    """

    def __init__(self, vbat=None, mass_share=0.5, decay='slow', pwm_hz=PWM_HZ):
        self.vbat = VBAT if vbat is None else vbat
        self.j = inertia_at_armature(mass_share)
        self.decay = decay
        self.pwm_hz = pwm_hz
        self.i = 0.0
        self.w = 0.0
        self.phase = 0.0
        self.i_min = 0.0
        self.i_max = 0.0
        self._acc_min = 1e9
        self._acc_max = -1e9

    @property
    def v(self):
        return self.w * R_WHEEL / GEAR_N

    def _applied(self, duty_abs, on):
        """Winding voltage right now, in the direction of drive."""
        if on:
            return self.vbat
        if self.decay == 'slow':
            return 0.0
        # Fast decay: body diodes push the current back into the supply, but
        # only while there is current to push.
        if abs(self.i) > 1e-6:
            return -(self.vbat + (2.0 * V_DIODE))
        return KE * self.w  # open circuit: terminals sit at the back-EMF

    def step(self, duty, dt, load_torque=0.0):
        duty = max(-1.0, min(1.0, duty))
        d = abs(duty)
        sign = 1.0 if duty >= 0.0 else -1.0
        r = R_A + R_DS_ON
        period = 1.0 / self.pwm_hz

        h = min(period / 200.0, 2.0e-6)
        n = max(1, int(round(dt / h)))
        h = dt / n

        for _ in range(n):
            prev_phase = self.phase
            self.phase = (self.phase + (h / period)) % 1.0
            if self.phase < prev_phase:
                # A PWM period just closed: publish its ripple. Measuring over the
                # control step instead would report whatever fraction of a period
                # happened to fall inside it.
                if self._acc_max > self._acc_min:
                    self.i_min, self.i_max = self._acc_min, self._acc_max
                self._acc_min, self._acc_max = 1e9, -1e9
            on = self.phase < d
            va = sign * self._applied(d, on)

            if (not on) and self.decay == 'fast' and abs(self.i) < 1e-6:
                self.i = 0.0  # open circuit, current stays at zero
            else:
                di = ((va - (self.i * r) - (KE * self.w)) / L_A) * h
                i_new = self.i + di
                # Fast decay cannot drive the current past zero: the diodes block.
                if self.decay == 'fast' and (not on) and (i_new * self.i < 0.0):
                    i_new = 0.0
                self.i = max(-I_LIMIT, min(I_LIMIT, i_new))

            self._acc_min = min(self._acc_min, self.i)
            self._acc_max = max(self._acc_max, self.i)

            t = (KT * self.i) - (B_M * self.w) - load_torque
            if abs(self.w) > 1.0:
                t -= math.copysign(T_COUL, self.w)
            elif abs(t) < T_COUL:
                t = 0.0
            self.w += (t / self.j) * h

        return self.v


# ----------------------------------------------------------------------------
# Transfer functions
# ----------------------------------------------------------------------------

def tf_duty_to_speed(vbat=None, mass_share=0.5):
    """Continuous-time duty -> wheel speed, as (num, den) polynomial coefficients.

                              KT * R_WHEEL / GEAR_N * VBAT
        V(s)/D(s) = -------------------------------------------------
                     (L*s + R)(J*s + b) + KT*KE

    Second order, but the two poles are three orders of magnitude apart, so it
    behaves as a first-order lag with tau = tau_mech.
    """
    vb = VBAT if vbat is None else vbat
    j = inertia_at_armature(mass_share)
    r = R_A + R_DS_ON
    num = [vb * KT * R_WHEEL / GEAR_N]
    den = [L_A * j, (L_A * B_M) + (r * j), (r * B_M) + (KT * KE)]
    return num, den


def tf_first_order(vbat=None, mass_share=0.5):
    """The first-order reduction of the above: K / (tau*s + 1)."""
    return dc_gain_duty_to_speed(vbat, mass_share), tau_mech(mass_share)


def tf_steer_to_lateral(v, d_la):
    """Look-ahead lateral error response to steering angle, at speed v.

    Kinematic bicycle, small angles:
        e_dot   = v * psi
        psi_dot = (v / L) * delta
        y_la    = e + d_la * psi

    so

        Y(s)/Delta(s) = (v/L) * (d_la * s + v) / s^2

    A double integrator with a zero at -v/d_la. Two consequences drive the whole
    steering design:

      * closed under a pure gain K, damping is zeta = (d_la/2) * sqrt(K/L),
        which does not contain v at all, while the natural frequency
        omega_n = v * sqrt(K/L) is directly proportional to it.
      * so the loop does not go unstable with speed because of the gain - it
        goes unstable because crossover rises with speed while the sensor and
        servo delay stays put, and phase margin is eaten at omega_c * T_delay.

    That is why the fix is scheduling on speed, not just turning the gain down.
    """
    num = [(v / WHEELBASE) * d_la, (v / WHEELBASE) * v]
    den = [1.0, 0.0, 0.0]
    return num, den


def corner_speed_limit(radius):
    """Fastest a corner of this radius can be taken, m/s. v = sqrt(mu*g*R)."""
    return math.sqrt(MU * G * max(radius, 1e-3))


def summary():
    lines = []
    a = lines.append
    a("25GA-370 derived parameters")
    a("  gear ratio          1:%.1f" % GEAR_N)
    a("  R (armature+bridge) %.2f ohm" % (R_A + R_DS_ON))
    a("  L                   %.2f mH" % (L_A * 1e3))
    a("  Ke = Kt             %.5f V*s/rad  (%.5f N*m/A)" % (KE, KT))
    a("  b                   %.3e N*m*s/rad" % B_M)
    a("  J at armature       %.3e kg*m^2  (rotor %.0f%%, car mass %.0f%%)"
      % (inertia_at_armature(),
         100.0 * J_MOT / inertia_at_armature(),
         100.0 * (MASS * 0.5 * R_WHEEL ** 2 / (GEAR_N ** 2 * GEAR_EFF)) / inertia_at_armature()))
    a("  tau_elec            %.2f ms" % (tau_elec() * 1e3))
    a("  tau_mech            %.0f ms   <- dominant pole" % (tau_mech() * 1e3))
    a("  DC gain             %.2f (m/s) per unit duty at %.1f V" % (dc_gain_duty_to_speed(), VBAT))
    a("  top speed           %.2f m/s at %.1f V (with rolling resistance)" % (top_speed(), VBAT))
    return "\n".join(lines)


if __name__ == "__main__":
    print(summary())
