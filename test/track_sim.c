/*
 * Host simulator for the NXP Cup racing firmware.
 *
 * Compiles the REAL track.c / racing_line.c / driver.c and drives a bicycle-model
 * car around a synthetic circuit, rendering a Pixy2-like 79x52 line-tracking frame
 * at every step (integer quantised, perspective projected, edges clipped out of
 * frame exactly like the real camera loses them).
 *
 * Checks:
 *   - the car never crosses a black line
 *   - it uses the width of the track (racing line, not centre line)
 *   - it is fast on straights and slow in corners
 *   - it drives straight through a small chicane
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "track.h"
#include "racing_line.h"
#include "driver.h"
#include "race_config.h"

#define PIXY_MAX_VECTORS 12

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* Circuit                                                             */
/* ------------------------------------------------------------------ */

typedef struct
{
    double curv; /* 1/cm, + = left turn in world frame */
    double len;  /* cm */
} TrackSeg;

#define CL_MAX 200000
static double g_cx[CL_MAX], g_cy[CL_MAX], g_cth[CL_MAX];
static int    g_cn = 0;
static double g_dumpFrom = -1.0, g_dumpTo = -1.0;
static double g_step = 0.5; /* cm between centre line samples */
static double g_halfW = 22.5;
/* How many vectors the Pixy2 splits one track edge into. Real hardware
 * merges smooth curves into a single long straight vector, which chords
 * across the arc - set this to 1 to reproduce that. */
static int g_maxChunks = 3;

static void build_centerline(const TrackSeg *segs, int n)
{
    double x = 0, y = 0, th = 0;
    int    i;

    g_cn = 0;
    for (i = 0; i < n; i++)
    {
        int    k, steps = (int)(segs[i].len / g_step);
        for (k = 0; k < steps && g_cn < CL_MAX; k++)
        {
            g_cx[g_cn] = x;
            g_cy[g_cn] = y;
            g_cth[g_cn] = th;
            g_cn++;
            x += g_step * cos(th);
            y += g_step * sin(th);
            th += g_step * segs[i].curv;
        }
    }
}

/* Nearest centre line index to a world point, searched around a hint. */
static int nearest_idx(double x, double y, int hint)
{
    int    best = hint, i;
    double bd = 1e18;
    int    lo = hint - 400, hi = hint + 4000;

    if (lo < 0) lo = 0;
    if (hi > g_cn) hi = g_cn;

    for (i = lo; i < hi; i++)
    {
        double dx = g_cx[i] - x, dy = g_cy[i] - y;
        double d  = dx * dx + dy * dy;
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}

/* Signed lateral offset from the centre line. + = left of the direction of travel. */
static double lateral_offset(double x, double y, int idx)
{
    double dx = x - g_cx[idx], dy = y - g_cy[idx];
    double th = g_cth[idx];
    return (-sin(th) * dx) + (cos(th) * dy);
}

/* ------------------------------------------------------------------ */
/* Camera model                                                        */
/* ------------------------------------------------------------------ */

typedef struct
{
    double f;      /* focal length in grid pixels */
    double h;      /* camera height, cm           */
    double horiz;  /* image row of the horizon    */
} Cam;

/* World point -> image. Returns false when it falls outside the frame. */
static bool project(const Cam *c, double fwd, double lat, double *u, double *v)
{
    if (fwd < 2.0) return false;
    *v = c->horiz + (c->f * c->h / fwd);
    *u = CAM_CENTER_X - (c->f * lat / fwd); /* +lat is to the left -> smaller column */
    if (*v < 0.0 || *v > (double)(PIXY_LINE_H - 1)) return false;
    if (*u < 0.0 || *u > (double)(PIXY_LINE_W - 1)) return false;
    return true;
}

/*
 * Renders one track edge into up to 3 segments, the way the Pixy2 breaks a curved
 * line into a small chain of vectors. Points are quantised to integers first.
 */
static int render_edge(const Cam *cam, double px, double py, double pth, int hint,
                       double side, TrkSegment *out, int maxOut)
{
    double ux[128], vy[128];
    int    np = 0, nseg = 0;
    int    idx;

    for (idx = hint; idx < g_cn && np < 128; idx += 2)
    {
        double th = g_cth[idx];
        double ex = g_cx[idx] - sin(th) * side * g_halfW;
        double ey = g_cy[idx] + cos(th) * side * g_halfW;
        double dx = ex - px, dy = ey - py;
        double fwd = cos(pth) * dx + sin(pth) * dy;
        double lat = -sin(pth) * dx + cos(pth) * dy;
        double u, v;

        if (fwd < 1.0) continue;
        if (fwd > 250.0) break;
        if (!project(cam, fwd, lat, &u, &v)) continue;

        u = floor(u + 0.5);
        v = floor(v + 0.5);
        if (np > 0 && u == ux[np - 1] && v == vy[np - 1]) continue;
        ux[np] = u; vy[np] = v; np++;
    }

    if (np < 2) return 0;

    /* Split the point chain into up to 3 straight vectors. */
    {
        int chunks = (np >= 12) ? 3 : ((np >= 6) ? 2 : 1);
        if (chunks > g_maxChunks) chunks = g_maxChunks;
        int c;
        for (c = 0; c < chunks && nseg < maxOut; c++)
        {
            int a = (np - 1) * c / chunks;
            int b = (np - 1) * (c + 1) / chunks;
            if (b <= a) continue;
            out[nseg].x0 = (float)ux[a];
            out[nseg].y0 = (float)vy[a];
            out[nseg].x1 = (float)ux[b];
            out[nseg].y1 = (float)vy[b];
            nseg++;
        }
    }
    return nseg;
}

/* ------------------------------------------------------------------ */
/* Vehicle                                                             */
/* ------------------------------------------------------------------ */

typedef struct
{
    double x, y, th;   /* pose, cm and rad         */
    double v;          /* speed, cm/s              */
    double halfCar;    /* half track width of car  */
} Car;

#define WHEELBASE      17.0
#define MAX_STEER_RAD  (30.0 * M_PI / 180.0)
#define GRIP_G         1.05  /* lateral grip, in g */

/* ------------------------------------------------------------------ *
 * Drivetrain: 2 x 25GA-370 through a DRV8833.
 *
 * These are not invented. They are derived in sim/plant.py from the published
 * motor numbers (12 V, 1000 rpm out, 0.12 A no load, 3.0 A stall) and validated
 * there against a PWM-resolved model that integrates the winding current at
 * 2 us. Keep the two in step: plant.py is the source of truth.
 *
 * The old model here was  v += (v_cmd*3.6 - v) * 6.0 * dt,  i.e. a 167 ms lag
 * onto a top speed of 3.6 m/s. The real machine is a 348 ms lag onto 2.0 m/s at
 * a DRV8833-legal 8 V, so the old model was flattering the car about twice over.
 * ------------------------------------------------------------------ */
#define MOT_GEAR_N     9.6
#define MOT_GEAR_EFF   0.70
#define MOT_R          4.36      /* armature + bridge on-resistance, ohm */
#define MOT_L          2.0e-3    /* H */
#define MOT_KE         0.011460  /* V*s/rad */
#define MOT_KT         0.011460  /* N*m/A   */
#define MOT_B          1.368e-6  /* N*m*s/rad */
#define MOT_TCOUL      1.6e-3    /* N*m at the armature */
#define MOT_ILIMIT     2.0       /* A, DRV8833 bridge limit */
#define MOT_VDIODE     0.9       /* V, body diode drop in fast decay */
#define MOT_PWM_HZ     1000.0    /* CTIMER0: 2343750 / 2343 */

#ifndef SIM_VBAT
#define SIM_VBAT       8.0       /* 2S LiPo. DRV8833 tops out at 10.8 V. */
#endif

/* The bridge is wired DIR pin + PWM pin, which gives coast-during-off going
 * forwards and brake-during-off in reverse. Set to 0 to model the slow-decay
 * wiring instead and see what it is worth. */
#ifndef SIM_FAST_DECAY_FWD
#define SIM_FAST_DECAY_FWD 1
#endif

#define CAR_MASS       1.20      /* kg, car with battery */
#define WHEEL_R        0.032     /* m */
#define REAR_TRACK     0.15      /* m */
#define C_ROLL         0.015
#define C_DRAG         0.0035    /* N/(m/s)^2 */

/* Rotating inertia referred to the road, so the whole drivetrain is one degree
 * of freedom: m_eff = m + 2*J_mot*N^2*eff/r^2 + 2*J_wheel/r^2. */
#define J_MOT          1.2e-6
#define J_WHEEL        1.5e-5
#define CAR_MASS_EFF   (CAR_MASS \
                        + (2.0 * J_MOT * MOT_GEAR_N * MOT_GEAR_N * MOT_GEAR_EFF \
                           / (WHEEL_R * WHEEL_R)) \
                        + (2.0 * J_WHEEL / (WHEEL_R * WHEEL_R)))

/* Latency the old model did not have at all: the frame the controller acts on
 * describes where the car WAS, and the servo does not step. */
#ifndef SIM_SENSE_DELAY_S
#define SIM_SENSE_DELAY_S 0.030
#endif
#ifndef SIM_SERVO_TAU_S
#define SIM_SERVO_TAU_S   0.060
#endif

/*
 * Average winding current over one PWM period, in closed form.
 *
 * The electrical time constant is 0.46 ms against a 348 ms mechanical one, so
 * within a PWM period the speed is a constant and the current waveform can be
 * solved instead of integrated. Over the on-time the current heads for
 * (V - E)/R; over the off-time it heads for -E/R when the bridge brakes, or
 * -(V + 2*Vd + E)/R when it coasts into the body diodes. Integrating both
 * exponentials and using i(t_z) = 0 at the zero crossing, everything cancels
 * except
 *
 *     <i> = ( I_on * t_on + I_off * min(t_z, t_off) ) / T
 *
 * which is exact in both continuous and discontinuous conduction.
 *
 * Braking shorts the winding, so current may reverse and conduction is always
 * continuous. Coasting cannot: the diodes block, and below about 30% duty the
 * winding spends part of every period open. That is the deadband the car has
 * going forwards today.
 */
static double motor_avg_current(double duty, double wArm, int fastDecay)
{
    double d    = fabs(duty);
    double sgn  = (duty >= 0.0) ? 1.0 : -1.0;
    double tau  = MOT_L / MOT_R;
    double T    = 1.0 / MOT_PWM_HZ;
    double tOn  = d * T;
    double tOff = T - tOn;
    double emf  = MOT_KE * wArm * sgn;
    double iOn  = (SIM_VBAT - emf) / MOT_R;
    double iOff, tZ, iAvg;

    if (d > 1.0) { d = 1.0; tOn = T; tOff = 0.0; }
    if (iOn >  MOT_ILIMIT) iOn =  MOT_ILIMIT;
    if (iOn < -MOT_ILIMIT) iOn = -MOT_ILIMIT;

    if (!fastDecay)
    {
        iOff = -emf / MOT_R;
        tZ   = tOff;
    }
    else
    {
        double a = exp(-tOn / tau);
        double b = exp(-tOff / tau);
        double den, iMin, iPeak;

        iOff = (-(SIM_VBAT + (2.0 * MOT_VDIODE)) - emf) / MOT_R;
        den  = 1.0 - (a * b);
        if (den < 1e-12) den = 1e-12;
        iMin = ((iOff * (1.0 - b)) + (iOn * b * (1.0 - a))) / den;

        if (iMin > 0.0)
        {
            tZ = tOff;                       /* continuous conduction */
        }
        else
        {
            iPeak = iOn * (1.0 - a);
            if (iPeak >  MOT_ILIMIT) iPeak =  MOT_ILIMIT;
            if (iPeak < -MOT_ILIMIT) iPeak = -MOT_ILIMIT;

            if (fabs(iPeak) < 1e-12)
                tZ = 0.0;                    /* never conducted at all */
            else if ((iPeak > 0.0) && (iOff < 0.0))
                tZ = tau * log(1.0 + (iPeak / -iOff));
            else if ((iPeak < 0.0) && (iOff > 0.0))
                tZ = tau * log(1.0 + (-iPeak / iOff));
            else
                tZ = tOff;

            if (tZ > tOff) tZ = tOff;
            if (tZ < 0.0)  tZ = 0.0;
        }
    }

    iAvg = ((iOn * tOn) + (iOff * tZ)) / T;
    if (iAvg >  MOT_ILIMIT) iAvg =  MOT_ILIMIT;
    if (iAvg < -MOT_ILIMIT) iAvg = -MOT_ILIMIT;
    return sgn * iAvg;
}

/* Wheel force in newtons from one motor at a given duty and road speed. */
static double motor_force(double cmdPercent, double vWheel)
{
    double duty = cmdPercent / 100.0;
    double wArm = vWheel * MOT_GEAR_N / WHEEL_R;
    double i, t;
    int    fast;

    if (duty >  1.0) duty =  1.0;
    if (duty < -1.0) duty = -1.0;

    fast = (duty >= 0.0) ? SIM_FAST_DECAY_FWD : 0;

    i = motor_avg_current(duty, wArm, fast);
    t = (MOT_KT * i) - (MOT_B * wArm);
    if (fabs(wArm) > 1.0)
        t -= (wArm > 0.0) ? MOT_TCOUL : -MOT_TCOUL;
    else if (fabs(t) < MOT_TCOUL)
        t = 0.0;

    return t * MOT_GEAR_N * MOT_GEAR_EFF / WHEEL_R;
}

/* ------------------------------------------------------------------ */

typedef struct
{
    const char *name;
    int    excursions;      /* control cycles with a wheel over a line */
    double worstOver;       /* deepest excursion, cm past the line     */
    double minClear;        /* smallest clearance to a line, cm        */
    double lapTime;
    double avgSpeed;
    double maxSpeed;
    double minSpeed;
    double dist;
    int    steps;
    int    oneEdgeFrames;
    int    lostFrames;
    int    chicaneFrames;
    int    finished;
    double peakSteerWin;
    double peakLatWin;
    double minSpeedWin;
    int    winFrames;
    double peakHN;
    double peakHF;
    double peakSev;
    double peakCurv;
} Result;

/* Centre line index window for the focused chicane measurement. */
static double g_dropRate = 0.0;   /* fraction of camera frames lost      */
static double g_blindFrom = -1.0; /* window where the camera sees nothing */
static double g_blindTo = -1.0;
static unsigned g_rng = 12345u;

static double frand(void)
{
    g_rng = (g_rng * 1103515245u) + 12345u;
    return (double)((g_rng >> 16) & 0x7FFFu) / 32768.0;
}

#define PROF_N 24
static double g_profSum[PROF_N];
static double g_profCnt[PROF_N];
static double g_profBias[PROF_N];
static double g_profWE[PROF_N];
static double g_profHF[PROF_N];
static int    g_profLo = -1, g_profHi = -1;

static int g_winLo = -1;
static int g_winHi = -1;

static Result run(const TrackSeg *segs, int nsegs, const Cam *cam, double halfW,
                  double startLat, const char *name, int verbose, double maxTime)
{
    Car        car;
    Result     r;
    TrkSegment allSegs[PIXY_MAX_VECTORS];
    DriveCmd   cmd;
    double     t = 0.0, dt = 0.004; /* 250 Hz control loop */
    int        hint = 0;
    int        frameDiv = 0;
    double     lapLen;
    double     servoAngle = 0.0;
    double     yawPrev = 0.0;

    /* Pose history, so the camera can be shown where the car WAS. Rendering the
     * frame from the current pose hands the controller information it cannot
     * have, and quietly hides every delay-driven stability problem. */
    #define POSE_HIST 128
    double hx[POSE_HIST], hy[POSE_HIST], hth[POSE_HIST];
    int    hidx[POSE_HIST];
    int    hw = 0;
    int    hLag = (int)((SIM_SENSE_DELAY_S / dt) + 0.5);
    int    hFilled = 0;


    memset(&r, 0, sizeof(r));
    r.name = name;
    r.minClear = 1e9;
    r.minSpeed = 1e9;
    r.minSpeedWin = 1e9;
    r.minSpeedWin = 1e9;

    g_halfW = halfW;
    build_centerline(segs, nsegs);

    car.x = g_cx[0] - sin(g_cth[0]) * startLat;
    car.y = g_cy[0] + cos(g_cth[0]) * startLat;
    car.th = g_cth[0];
    car.v = 0.0;
    car.halfCar = 7.0;

    Driver_Init();

    while (t < maxTime)
    {
        bool    fresh;
        uint8_t n = 0;
        int     idx;
        double  lat, clear;

        idx  = nearest_idx(car.x, car.y, hint);
        hint = idx;

        /* Record where the car is now, and pick out where it was one sensor
         * delay ago - that is the pose the next camera frame describes. */
        hx[hw] = car.x; hy[hw] = car.y; hth[hw] = car.th; hidx[hw] = idx;
        hw = (hw + 1) % POSE_HIST;
        if (hFilled < POSE_HIST) hFilled++;

        /* --- camera frame at 60 Hz, control loop at 250 Hz --- */
        fresh = false;
        if (++frameDiv >= 4)
        {
            int k;
            int hr = (hw - 1 - hLag + (2 * POSE_HIST)) % POSE_HIST;
            double sx, sy, sth;
            int    sidx;

            if (hFilled > hLag)
            {
                sx = hx[hr]; sy = hy[hr]; sth = hth[hr]; sidx = hidx[hr];
            }
            else
            {
                sx = car.x; sy = car.y; sth = car.th; sidx = idx;
            }

            frameDiv = 0;
            k = render_edge(cam, sx, sy, sth, sidx, +1.0, allSegs, 4);
            n = (uint8_t)k;
            k = render_edge(cam, sx, sy, sth, sidx, -1.0, allSegs + n,
                            PIXY_MAX_VECTORS - n);
            n = (uint8_t)(n + k);
            fresh = true;

            /* fault injection */
            if (frand() < g_dropRate)
            {
                fresh = false;   /* camera returned busy / a bad packet */
                n = 0;
            }
            if ((g_blindFrom >= 0.0) && (idx >= (int)(g_blindFrom / g_step)) &&
                (idx <= (int)(g_blindTo / g_step)))
            {
                n = 0;           /* camera sees nothing at all */
            }
        }

        Driver_Step(fresh, allSegs, n, (float)dt, &cmd);

        if (fresh)
        {
            const DriveState *st = Driver_State();
            if (!st->track.haveTrack) r.lostFrames++;
            else if (!st->track.bothEdges) r.oneEdgeFrames++;
            if (st->line.chicane) r.chicaneFrames++;
        }

        /* --- vehicle --- */
        {
            /* The servo does not step. The firmware already rate-limits its
             * command; this is the linkage and the servo loop on top of it. */
            double delta, vMs, vL, vR, fL, fR, fNet, yaw;

            servoAngle += (cmd.steer - servoAngle) * (dt / (SIM_SERVO_TAU_S + dt));

            /* Firmware convention: + steer = turn RIGHT. World frame here has
             * increasing heading = turn LEFT, hence the negation. */
            delta = -(servoAngle / 100.0) * MAX_STEER_RAD;

            /* Two independent motors, so the wheels turn at different speeds
             * through a corner and each sees its own back-EMF. That is what
             * makes torque vectoring cost something as well as buy something. */
            vMs = car.v / 100.0;
            vL  = vMs - (yawPrev * REAR_TRACK * 0.5);
            vR  = vMs + (yawPrev * REAR_TRACK * 0.5);

            fL = motor_force(cmd.left, vL);
            fR = motor_force(cmd.right, vR);

            fNet = fL + fR;
            if (vMs > 0.01)
                fNet -= C_ROLL * CAR_MASS * 9.81;
            else if (vMs < -0.01)
                fNet += C_ROLL * CAR_MASS * 9.81;
            fNet -= C_DRAG * vMs * fabs(vMs);

            vMs += (fNet / CAR_MASS_EFF) * dt;
            car.v = vMs * 100.0;
            if (car.v < 0.0) car.v = 0.0;

            yaw = (car.v / WHEELBASE) * tan(delta);

            /* Grip limit: beyond it the car understeers instead of turning. */
            if (car.v > 1.0)
            {
                double aLatMax = GRIP_G * 981.0;
                double yawMax  = aLatMax / car.v;
                if (yaw > yawMax) yaw = yawMax;
                if (yaw < -yawMax) yaw = -yawMax;
            }
            yawPrev = yaw;

            car.th += yaw * dt;
            car.x += car.v * cos(car.th) * dt;
            car.y += car.v * sin(car.th) * dt;
        }

        /* --- scoring --- */
        lat   = lateral_offset(car.x, car.y, idx);
        clear = halfW - fabs(lat) - car.halfCar;
        if (clear < r.minClear) r.minClear = clear;
        if (clear < 0.0)
        {
            r.excursions++;
            if (-clear > r.worstOver) r.worstOver = -clear;
        }

        if (t > START_DELAY_MS / 1000.0)
        {
            if (car.v > r.maxSpeed) r.maxSpeed = car.v;
            if (car.v < r.minSpeed) r.minSpeed = car.v;
            r.dist += car.v * dt;
        }

        if (fresh && t >= g_dumpFrom && t <= g_dumpTo)
        {
            const DriveState *st = Driver_State();
            int q;
            printf("-- t=%.3f pose lat=%+.2f  segs=%d\n", t, lat, (int)n);
            for (q = 0; q < (int)n; q++)
                printf("     seg%d (%.0f,%.0f)->(%.0f,%.0f)\n", q,
                       allSegs[q].x0, allSegs[q].y0, allSegs[q].x1, allSegs[q].y1);
            for (q = 0; q < TRK_ROWS; q++)
                printf("     row%d y=%2.0f v=%d L=%7.1f R=%7.1f c=%6.1f w=%6.1f sL=%d sR=%d\n",
                       q, st->track.y[q], st->track.valid[q], st->track.xl[q],
                       st->track.xr[q], st->track.center[q], st->track.width[q],
                       st->track.sawL[q], st->track.sawR[q]);
            printf("     hN=%+.3f hF=%+.3f nValid=%d tgt=%.1f la=%d steer=%.1f\n",
                   st->track.headNear, st->track.headFar, st->track.nValid,
                   st->line.targetX, st->line.laRow, cmd.steer);
        }

        if (verbose && (r.steps % 25 == 0))
        {
            const DriveState *st = Driver_State();
            printf("  t=%5.2f lat=%+6.2f clr=%+5.2f v=%5.1f steer=%+6.1f "
                   "hN=%+5.2f hF=%+5.2f rows=%d bias=%+.2f%s%s\n",
                   t, lat, clear, car.v, cmd.steer,
                   st->track.headNear, st->track.headFar, st->track.nValid,
                   st->line.bias,
                   st->line.chicane ? " CHI" : "",
                   cmd.braking ? " BRK" : "");
        }

        if ((g_winLo >= 0) && (idx >= g_winLo) && (idx <= g_winHi))
        {
            double as = fabs(cmd.steer);
            double al = fabs(lat);
            if (as > r.peakSteerWin) r.peakSteerWin = as;
            if (al > r.peakLatWin) r.peakLatWin = al;
            if (car.v < r.minSpeedWin) r.minSpeedWin = car.v;
            { const DriveState *ds = Driver_State();
              double ah=fabs(ds->track.headNear), af=fabs(ds->track.headFar);
              if (ah > r.peakHN) r.peakHN = ah;
              if (af > r.peakHF) r.peakHF = af;
              if (ds->severity > r.peakSev) r.peakSev = ds->severity;
              if (fabs(ds->track.curv) > r.peakCurv) r.peakCurv = fabs(ds->track.curv); }
            r.winFrames++;
        }

        if ((g_profLo >= 0) && (idx >= g_profLo) && (idx < g_profHi))
        {
            int bi = (int)((double)(idx - g_profLo) * PROF_N / (double)(g_profHi - g_profLo));
            if (bi >= 0 && bi < PROF_N) { const DriveState *ps = Driver_State();
                g_profSum[bi] += lat; g_profCnt[bi] += 1.0;
                g_profBias[bi] += ps->line.bias; g_profWE[bi] += ps->line.wEntry;
                g_profHF[bi] += ps->track.headFar; }
        }

        r.steps++;
        t += dt;

        /* Finished when the car reaches the end of the circuit polyline. */
        if (idx >= g_cn - 300) { r.finished = 1; break; }
    }

    r.lapTime = t;
    r.avgSpeed = (t > 0.0) ? r.dist / t : 0.0;
    if (r.minSpeed > 1e8) r.minSpeed = 0.0;
    if (r.minSpeedWin > 1e8) r.minSpeedWin = 0.0;
    if (r.minSpeedWin > 1e8) r.minSpeedWin = 0.0;

    return r;
}

/* ------------------------------------------------------------------ */

static void report(const Result *r)
{
    printf("%-26s %-8s laptime=%6.2fs  dist=%6.0fcm  avg=%5.1f max=%5.1f cm/s\n",
           r->name, r->finished ? "FINISHED" : "*DNF*", r->lapTime, r->dist,
           r->avgSpeed, r->maxSpeed);
    printf("%-26s minClearance=%+6.2f cm   excursions=%d (worst %.2f cm over)\n",
           "", r->minClear, r->excursions, r->worstOver);
    printf("%-26s oneEdge=%d  lost=%d  chicaneFrames=%d\n\n",
           "", r->oneEdgeFrames, r->lostFrames, r->chicaneFrames);
}

/*
 * Steady-state duty -> speed of the model above, so it can be checked against
 * sim/plant.py, which is where these parameters were identified and validated
 * against a PWM-resolved simulation. The two are separate implementations of the
 * same equations; if they ever disagree, one of them has drifted.
 */
static void dump_motor_map(void)
{
    int i;

    printf("duty  speed m/s   (compare: python sim/plant.py map)\n");
    for (i = 0; i <= 10; i++)
    {
        double duty = i * 10.0;
        double v = 0.0;
        int    k;

        for (k = 0; k < 200000; k++)
        {
            double f = 2.0 * motor_force(duty, v);

            if (v > 0.01) f -= C_ROLL * CAR_MASS * 9.81;
            f -= C_DRAG * v * fabs(v);
            v += (f / CAR_MASS_EFF) * 1.0e-4;
            if (v < 0.0) v = 0.0;
        }
        printf("%4.0f  %9.4f\n", duty, v);
    }
}

int main(int argc, char **argv)
{
    Cam cam;
    int verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);

    if (argc > 1 && strcmp(argv[1], "-motormap") == 0)
    {
        dump_motor_map();
        return 0;
    }
    int only = -1;
    if (argc > 3 && strcmp(argv[1], "-d") == 0)
    {
        g_dumpFrom = atof(argv[2]);
        g_dumpTo = atof(argv[3]);
        only = (argc > 4) ? atoi(argv[4]) : 1;
    }

    /* Pixy2: 60 x 40 degree FOV on a 79 x 52 grid -> f ~ 68 grid px.
     * Mast 18 cm high, tilted down so the frame covers ~22 cm to ~1.7 m ahead. */
    cam.f = 68.0;
    cam.h = 18.0;
    cam.horiz = -4.0;

    if (argc > 4 && strcmp(argv[1], "-one") == 0)
    {
        TrackSeg t[] = {
            {0.0, 250.0},
            {1.0 / 60.0, 60.0 * M_PI},
            {0.0, 180.0},
            {-1.0 / 90.0, 90.0 * (M_PI / 2)},
            {0.0, 120.0},
            {1.0 / 75.0, 75.0 * (M_PI / 2)},
            {0.0, 150.0},
        };
        Cam cm;
        Result r;

        cm.f = atof(argv[2]);
        cm.h = atof(argv[3]);
        cm.horiz = atof(argv[4]);
        r = run(t, 7, &cm, (argc > 5) ? atof(argv[5]) : 22.5, 0.0, "single", 1, 40.0);
        report(&r);
        return 0;
    }

    /* -envelope runs only the sanely aimed mountings, which is the subset a
     * tuning search actually has to respect. It is 18 runs instead of 162, so it
     * is cheap enough to sit inside a search loop rather than being checked
     * afterwards - and checking afterwards is how a search ends up selling the
     * camera robustness to buy a second. */
    if (argc > 1 && (strcmp(argv[1], "-sweep") == 0 ||
                     strcmp(argv[1], "-envelope") == 0))
    {
        int onlyEnv = (strcmp(argv[1], "-envelope") == 0);
        static const double F[]  = { 55.0, 68.0, 80.0 };
        static const double H[]  = { 12.0, 18.0, 26.0 };
        static const double HZ[] = { -10.0, -6.0, -3.0 };
        static const double HW[] = { 22.5, 27.5, 30.0 };
        static const double RAD[] = { 60.0, 80.0 };
        int a, b, c, d, e;
        int nR = 0, finR = 0, cleanR = 0;
        int nS = 0, finS = 0, cleanS = 0;
        double worstR = 1e9, worstS = 1e9;

        printf("=== robustness sweep ===\n");
        printf("camera focal 55-80 px, height 12-26 cm, horizon row -10..-3,\n");
        printf("track 45/55/60 cm, corner radius 60 and 80 cm.\n\n");
        printf("A mounting is IN ENVELOPE when the frame covers roughly 15-35 cm at the\n");
        printf("bottom and 80-250 cm at the top - i.e. a sanely aimed camera.\n\n");
        printf("problem cases:\n");

        for (a = 0; a < 3; a++)
        for (b = 0; b < 3; b++)
        for (c = 0; c < 3; c++)
        for (d = 0; d < 3; d++)
        for (e = 0; e < 2; e++)
        {
            TrackSeg t[] = {
                {0.0, 250.0},
                {1.0 / RAD[e], RAD[e] * M_PI},
                {0.0, 180.0},
                {-1.0 / 90.0, 90.0 * (M_PI / 2)},
                {0.0, 120.0},
                {1.0 / 75.0, 75.0 * (M_PI / 2)},
                {0.0, 150.0},
            };
            Cam cm;
            Result r;
            char nm[80];
            double dNear, dFar;
            int inEnv;

            cm.f = F[a]; cm.h = H[b]; cm.horiz = HZ[c];
            dNear = cm.f * cm.h / (51.0 - cm.horiz);
            dFar  = cm.f * cm.h / (0.0 - cm.horiz);
            /* Both black lines must fit inside the frame at mid height. Note the
             * focal length cancels: this is purely about camera height versus how
             * far down the picture is tilted. */
            {
                double halfViewMid = (0.5 * (PIXY_LINE_W - 1)) * cm.h / (26.0 - cm.horiz);
                inEnv = (dNear >= 15.0 && dNear <= 35.0 && dFar >= 80.0 && dFar <= 250.0 &&
                         halfViewMid >= HW[d]);
            }

            if (onlyEnv && !inEnv)
            {
                continue;
            }

            sprintf(nm, "f=%.0f h=%.0f hz=%+.0f track=%.0f R=%.0f", F[a], H[b], HZ[c],
                    2 * HW[d], RAD[e]);
            r = run(t, 7, &cm, HW[d], 0.0, nm, 0, 40.0);

            if (inEnv) { nR++; if (r.finished) finR++; if (r.excursions == 0) cleanR++;
                         if (r.minClear < worstR) worstR = r.minClear; }
            else       { nS++; if (r.finished) finS++; if (r.excursions == 0) cleanS++;
                         if (r.minClear < worstS) worstS = r.minClear; }

            /* -envelope lists all 18, because when a tuning fails on some of them
             * the useful question is which ones - that is what tells you whether
             * your own bracket is anywhere near the ones that broke. */
            if (onlyEnv || r.excursions > 0 || !r.finished)
                printf("  %-11s %-38s finished=%-4s excursions=%-5d minClear=%+.2fcm%s\n",
                       inEnv ? "IN-ENVELOPE" : "stress", nm,
                       r.finished ? "yes" : "NO", r.excursions, r.minClear,
                       (onlyEnv && r.excursions == 0) ? "  ok" : "");
        }

        printf("\nIN ENVELOPE : %d configurations, %d finished, %d with zero excursions, worst clearance %+.2f cm\n",
               nR, finR, cleanR, worstR);
        printf("stress      : %d configurations, %d finished, %d with zero excursions, worst clearance %+.2f cm\n",
               nS, finS, cleanS, worstS);
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "-line") == 0)
    {
        /* One long constant radius corner with a straight either side.
         * Positive lateral offset is to the LEFT of the direction of travel, so in
         * this left hand corner positive means the inside. */
        TrackSeg t[] = {
            {0.0, 260.0},
            {1.0 / 85.0, 85.0 * M_PI},
            {0.0, 260.0},
        };
        Cam cm;
        Result r;
        int i;
        double half = 22.5;

        cm.f = 68.0; cm.h = 18.0; cm.horiz = -4.0;
        if (argc > 2) g_maxChunks = atoi(argv[2]);
        for (i = 0; i < PROF_N; i++) { g_profSum[i] = 0.0; g_profCnt[i] = 0.0; g_profBias[i]=0.0; g_profWE[i]=0.0; g_profHF[i]=0.0; }
        g_profLo = (int)(180.0 / g_step);
        g_profHi = (int)((260.0 + (85.0 * M_PI) + 120.0) / g_step);

        r = run(t, 3, &cm, half, 0.0, "racing line", 0, 40.0);
        g_profLo = -1;

        printf("=== the line through a 170 cm diameter left hander ===\n");
        printf("track is 45 cm wide. + is toward the INSIDE of the corner, - toward the OUTSIDE.\n");
        printf("each row is a slice of the lap, from the approach to the exit.\n\n");
        for (i = 0; i < PROF_N; i++)
        {
            double v = (g_profCnt[i] > 0.0) ? (g_profSum[i] / g_profCnt[i]) : 0.0;
            int    col;
            char   bar[49];
            int    j;
            const char *phase;

            for (j = 0; j < 48; j++) bar[j] = ' ';
            bar[48] = 0;
            bar[24] = '|';
            col = 24 + (int)(v / half * 22.0);
            if (col < 0) col = 0;
            if (col > 47) col = 47;
            bar[col] = '#';

            if (i < 5)       phase = "approach (straight)";
            else if (i < 8)  phase = "turn in";
            else if (i < 15) phase = "apex";
            else if (i < 19) phase = "exit";
            else             phase = "straight again";

            printf("%s %+6.1fcm bias=%+5.2f wEntry=%4.2f hF=%+5.2f %s\n", bar, v,
                   g_profBias[i]/(g_profCnt[i]>0?g_profCnt[i]:1),
                   g_profWE[i]/(g_profCnt[i]>0?g_profCnt[i]:1),
                   g_profHF[i]/(g_profCnt[i]>0?g_profCnt[i]:1), phase);
        }
        printf("\n%48s\n", "outside      centre      inside");
        printf("excursions=%d  minClearance=%+.2f cm  laptime=%.2fs\n",
               r.excursions, r.minClear, r.lapTime);
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "-chord") == 0)
    {
        /* The same corners seen two ways: with the camera splitting each edge into
         * up to three vectors, and with it merging each edge into one long straight
         * vector that chords across the curve - which is what the real Pixy2 does to
         * a smooth corner. */
        int c;

        printf("=== effect of the Pixy2 merging a curve into one straight vector ===\n");
        printf("%-22s %-9s %-10s %-9s %s\n", "vectors per edge", "finished", "total lap",
               "minClear", "excursions");
        for (c = 3; c >= 1; c--)
        {
            TrackSeg t[] = {
                {0.0, 250.0},
                {1.0 / 70.0, 70.0 * M_PI},
                {0.0, 200.0},
                {-1.0 / 90.0, 90.0 * (M_PI / 2)},
                {0.0, 150.0},
            };
            Cam cm;
            Result r;
            double total;

            cm.f = 68.0; cm.h = 18.0; cm.horiz = -4.0;
            g_maxChunks = c;
            r = run(t, 5, &cm, 22.5, 0.0, "chord", 0, 40.0);
            total = r.lapTime;
            printf("%-22d %-9s %-10.2f %+8.2f  %d\n", c, r.finished ? "yes" : "NO",
                   total, r.minClear, r.excursions);
        }
        g_maxChunks = 3;
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "-fault") == 0)
    {
        /* Blind windows are given in cm along the circuit, so the same piece of road
         * is blacked out no matter how quickly the car gets there. The corner starts
         * at 250 cm. */
        struct { const char *name; double drop; double bFrom; double bTo; } k[] = {
            { "clean run",                  0.00,  -1.0,  -1.0 },
            { "20% frames lost",            0.20,  -1.0,  -1.0 },
            { "50% frames lost",            0.50,  -1.0,  -1.0 },
            { "80% frames lost",            0.80,  -1.0,  -1.0 },
            { "blind 20cm into a corner",   0.00, 330.0, 350.0 },
            { "blind 60cm into a corner",   0.00, 330.0, 390.0 },
            { "blind 150cm through it",     0.00, 330.0, 480.0 },
            { "40% lost + blind 40cm",      0.40, 330.0, 370.0 },
        };
        int i;

        printf("=== failsafe: what happens when the camera stops helping ===\n");
        printf("%-26s %-9s %-8s %-10s %s\n", "fault", "finished", "excurs", "minClear", "verdict");
        for (i = 0; i < 8; i++)
        {
            TrackSeg t[] = {
                {0.0, 250.0},
                {1.0 / 80.0, 80.0 * M_PI},
                {0.0, 180.0},
                {-1.0 / 90.0, 90.0 * (M_PI / 2)},
                {0.0, 200.0},
            };
            Cam cm;
            Result r;

            cm.f = 68.0; cm.h = 18.0; cm.horiz = -4.0;
            g_rng = 12345u;
            g_dropRate = k[i].drop;
            g_blindFrom = k[i].bFrom;
            g_blindTo = k[i].bTo;
            r = run(t, 5, &cm, 22.5, 0.0, k[i].name, 0, 40.0);
            printf("%-26s %-9s %-8d %+9.2f  %s\n", k[i].name, r.finished ? "yes" : "no",
                   r.excursions, r.minClear,
                   (r.excursions == 0) ? "stayed inside the lines"
                                       : "TOUCHED A LINE");
        }
        g_dropRate = 0.0;
        g_blindFrom = -1.0;
        g_blindTo = -1.0;
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "-chicane") == 0)
    {
        struct { const char *name; double rad; double l1; double l2; } k[] = {
            { "tiny wiggle",    300.0, 40.0,  80.0 },
            { "small chicane",  180.0, 55.0, 110.0 },
            { "medium chicane", 110.0, 65.0, 130.0 },
            { "real S bend",     90.0, 85.0, 170.0 },
        };
        int i;

        printf("=== chicane handling: are the little ones ignored? ===\n");
        printf("%-16s %-9s %-9s %-8s %-7s %-7s %-6s %s\n", "feature", "pk|steer|", "pk|lat|cm", "minspeed", "pk hN", "pk hF", "excurs", "behaviour");
        for (i = 0; i < 4; i++)
        {
            TrackSeg t[] = {
                {0.0, 300.0},
                {1.0 / k[i].rad, k[i].l1},
                {-1.0 / k[i].rad, k[i].l2},
                {1.0 / k[i].rad, k[i].l1},
                {0.0, 300.0},
            };
            Cam cm;
            Result r;

            cm.f = 68.0; cm.h = 18.0; cm.horiz = -4.0;
            g_winLo = (int)(290.0 / g_step);
            g_winHi = (int)((300.0 + (k[i].l1 * 2.0) + k[i].l2 + 40.0) / g_step);
            r = run(t, 5, &cm, 22.5, 0.0, k[i].name, 0, 40.0);
            printf("%-16s %-9.1f %-9.2f %-8.0f %-7.2f %-7.2f %-6.2f %s\n", k[i].name, r.peakSteerWin,
                   r.peakLatWin, r.minSpeedWin, r.peakHN, r.peakHF, (double)r.excursions,
                   (r.peakSteerWin < 12.0) ? "ignored - drove straight through"
                                           : "steered through it");
        }
        g_winLo = -1;
        g_winHi = -1;
        return 0;
    }

    printf("=== NXP Cup racing firmware - closed loop simulation ===\n");
    printf("camera f=%.0f h=%.0fcm horizon=%.0f   SPEED_MAX=%.0f  half-track=%.1fcm\n\n",
           cam.f, cam.h, cam.horiz, (double)SPEED_MAX, 22.5);

    /* --- 1. long straight then a fast sweeper --- */
    {
        TrackSeg t[] = {
            {0.0, 300.0},
            {1.0 / 90.0, 90.0 * M_PI},   /* 180 deg, R = 90 cm */
            {0.0, 300.0},
            {-1.0 / 90.0, 90.0 * M_PI},
            {0.0, 100.0},
        };
        if (only < 0 || only == 1) { Result r = run(t, 5, &cam, 22.5, 0.0, "oval, R=90cm", verbose, 40.0); report(&r); }
    }

    /* --- 2. tight hairpin --- */
    {
        TrackSeg t[] = {
            {0.0, 200.0},
            {1.0 / 55.0, 55.0 * M_PI},   /* tight left  180, near the car's limit  */
            {0.0, 200.0},
            {-1.0 / 80.0, 80.0 * M_PI},  /* tight right 180, near the car's limit  */
            {0.0, 100.0},
        };
        if (only < 0 || only == 2) { Result r = run(t, 5, &cam, 22.5, 0.0, "tight 180s, L55 / R80", verbose, 40.0); report(&r); }
    }

    /* --- 3. small chicane: should be ignored and driven straight through --- */
    {
        TrackSeg t[] = {
            {0.0, 250.0},
            {1.0 / 200.0, 60.0},    /* gentle left  */
            {-1.0 / 200.0, 120.0},  /* gentle right */
            {1.0 / 200.0, 60.0},    /* back to line */
            {0.0, 250.0},
            {1.0 / 70.0, 70.0 * M_PI},
            {0.0, 150.0},
            {-1.0 / 95.0, 95.0 * M_PI},
            {0.0, 100.0},
        };
        if (only < 0 || only == 3) { Result r = run(t, 9, &cam, 22.5, 0.0, "chicane + sweepers", verbose, 45.0); report(&r); }
    }

    /* --- 4. mixed circuit, started off centre against the left line --- */
    {
        TrackSeg t[] = {
            {0.0, 260.0},
            {1.0 / 60.0, 60.0 * (M_PI / 2)},
            {0.0, 120.0},
            {-1.0 / 85.0, 85.0 * (M_PI / 2)},
            {0.0, 80.0},
            {-1.0 / 85.0, 85.0 * (M_PI / 2)},
            {0.0, 200.0},
            {1.0 / 80.0, 80.0 * M_PI},
            {0.0, 140.0},
            {1.0 / 55.0, 55.0 * (M_PI / 2)},
            {0.0, 150.0},
        };
        if (only < 0 || only == 4) { Result r = run(t, 11, &cam, 22.5, 12.0, "mixed circuit, off-centre start", verbose, 60.0); report(&r); }
    }

    /* --- 5. narrower track, same code --- */
    {
        TrackSeg t[] = {
            {0.0, 250.0},
            {1.0 / 60.0, 60.0 * M_PI},
            {0.0, 250.0},
            {-1.0 / 85.0, 85.0 * M_PI},
            {0.0, 100.0},
        };
        if (only < 0 || only == 5) { Result r = run(t, 5, &cam, 17.5, 0.0, "narrow track (35cm)", verbose, 45.0); report(&r); }
    }

    return 0;
}
