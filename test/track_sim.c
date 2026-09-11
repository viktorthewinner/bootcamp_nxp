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
#include "features.h"
#include "classifier.h"

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

/* ------------------------------------------------------------------ */
/* Crossings                                                           */
/*
 * A crossing is a second track cutting across this one. In the world it is a band
 * of a given width, at a given angle, centred on a point along the centre line:
 * inside that band our own black lines are simply not there, and along each of its
 * two boundaries runs one of the crossing track's own edges.
 *
 * Describing it as a band rather than as a range of centre line indices is what
 * lets it sit at an angle, and lets more than one of them exist on a circuit.
 */
/* ------------------------------------------------------------------ */

#define SIM_MAX_XING 4

typedef struct
{
    double at;  /* cm along the centre line, the middle of the crossing   */
    double w;   /* width of the crossing track, cm                        */
    double bar; /* how far its edges reach out beyond ours, cm            */
    int    far; /* 0 = the camera never reports the far side of our edges */
} SimXing;

static SimXing g_xing[SIM_MAX_XING];
static int     g_nXing = 0;
static int     g_isecTrace = 0; /* 1 print detections, 2 print every frame near one */

static void xing_clear(void)
{
    g_nXing = 0;
}

static void xing_add(double at, double w, double bar, int far)
{
    if (g_nXing >= SIM_MAX_XING) return;
    g_xing[g_nXing].at  = at;
    g_xing[g_nXing].w   = w;
    g_xing[g_nXing].bar = bar;
    g_xing[g_nXing].far = far;
    g_nXing++;
}

/*
 * The crossing's own axis at index k: c is its centre, m the direction our track
 * runs through it, mp the direction its two edges run in.
 *
 * Taken from the tangent at that point, so a crossing dropped in the middle of a
 * corner is square to the track THERE - which is what a tile laid on a bend looks
 * like, and is the case an index-range model gets wrong.
 */
static void xing_frame(int k, double *cx, double *cy, double *mx, double *my,
                       double *px, double *py)
{
    int    i = (int)(g_xing[k].at / g_step);
    double th;

    if (i < 0) i = 0;
    if (i >= g_cn) i = g_cn - 1;

    th  = g_cth[i];
    *cx = g_cx[i];
    *cy = g_cy[i];
    *mx = cos(th);
    *my = sin(th);
    *px = -(*my);
    *py = *mx;
}

/*
 * Is this world point inside the band a crossing cuts out of our black lines?
 *
 * idx says which part of the circuit the point belongs to, and it is not
 * decoration: the band is a strip, and a strip is infinite. On a hairpin the track
 * turns round and comes back into the same strip a metre later, and without the
 * index window the crossing would eat a piece of black line on the far side of the
 * corner as well as the piece it is actually on.
 */
static int in_any_xing(int idx, double x, double y)
{
    int k;

    for (k = 0; k < g_nXing; k++)
    {
        double cx, cy, mx, my, px, py;
        double d, along;

        along = fabs((idx * g_step) - g_xing[k].at);
        if (along > ((0.5 * g_xing[k].w) + g_halfW + 20.0))
        {
            continue; /* a different part of the circuit that the strip reaches */
        }

        xing_frame(k, &cx, &cy, &mx, &my, &px, &py);
        d = ((x - cx) * mx) + ((y - cy) * my);
        if (fabs(d) <= (0.5 * g_xing[k].w))
        {
            return 1;
        }
    }
    return 0;
}

/* Does any crossing in front of the car want its far side suppressed? */
static int xing_far_suppressed(void)
{
    int k;

    for (k = 0; k < g_nXing; k++)
    {
        if (!g_xing[k].far)
        {
            return 1;
        }
    }
    return 0;
}

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

/* Turns one run of image points into up to maxChunks straight vectors, the way the
 * Pixy2 breaks a curved line into a short chain. */
static int emit_chunks(const double *ux, const double *vy, int np, int maxChunks,
                       TrkSegment *out, int maxOut)
{
    int nseg = 0, chunks, c;

    if (np < 2 || maxOut < 1) return 0;

    chunks = (np >= 12) ? 3 : ((np >= 6) ? 2 : 1);
    if (chunks > g_maxChunks) chunks = g_maxChunks;
    if (chunks > maxChunks)   chunks = maxChunks;

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
    return nseg;
}

/*
 * Renders one track edge into the frame.
 *
 * A crossing breaks it: the piece the car is following stops at the near boundary,
 * and the same line picks up again past the far one. Both pieces are emitted, and
 * that is not a detail - the far one is what tells the detector it is looking at a
 * crossing rather than at an edge that has simply run out of frame. Set a
 * crossing's far flag to 0 to model a camera that stops reporting it.
 */
static int render_edge(const Cam *cam, double px, double py, double pth, int hint,
                       double side, TrkSegment *out, int maxOut)
{
    double ux[128], vy[128];
    int    np = 0, nseg = 0;
    int    idx;
    int    past = 0; /* a piece has already been emitted, so this is the far one */

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

        /* Our black line is not there across a crossing. Close the piece that was
         * being collected and start a fresh one on the far side. */
        if (g_nXing > 0 && in_any_xing(idx, ex, ey))
        {
            if (np > 0)
            {
                nseg += emit_chunks(ux, vy, np, 3, out + nseg, maxOut - nseg);
                np = 0;
                past = 1;
            }
            if (xing_far_suppressed()) break;
            continue;
        }

        if (!project(cam, fwd, lat, &u, &v)) continue;

        u = floor(u + 0.5);
        v = floor(v + 0.5);
        if (np > 0 && u == ux[np - 1] && v == vy[np - 1]) continue;
        ux[np] = u; vy[np] = v; np++;
    }

    /* The far piece is small and far away, so the camera would report it as one
     * vector, not a chain. That also keeps the whole frame inside the twelve
     * vectors the Pixy2 hands over. */
    nseg += emit_chunks(ux, vy, np, past ? 1 : 3, out + nseg, maxOut - nseg);
    return nseg;
}

/*
 * One edge of the crossing track: a straight line running out past one of our own
 * black lines, starting exactly where that line stopped.
 *
 * k picks the crossing, sb which of its two boundaries (-1 near, +1 far), and side
 * which of our edges it runs out from (+1 left, -1 right).
 */
static int render_bar(const Cam *cam, double px, double py, double pth,
                      int k, double sb, double side, TrkSegment *out, int maxOut)
{
    double ux[64], vy[64];
    int    np = 0, j;
    double cx, cy, mx, my, ppx, ppy;
    double t0;

    if (maxOut < 1) return 0;

    xing_frame(k, &cx, &cy, &mx, &my, &ppx, &ppy);

    /* It starts where our own black line stopped: half a track out to the side. */
    t0 = side * g_halfW;

    for (j = 0; j <= 40 && np < 64; j++)
    {
        double t = t0 + (side * g_xing[k].bar * (double)j / 40.0);
        double bx = cx + (0.5 * g_xing[k].w * sb * mx) + (t * ppx);
        double by = cy + (0.5 * g_xing[k].w * sb * my) + (t * ppy);
        double dx = bx - px, dy = by - py;
        double fwd = cos(pth) * dx + sin(pth) * dy;
        double lat = -sin(pth) * dx + cos(pth) * dy;
        double u, v;

        if (fwd < 1.0) continue;
        if (fwd > 250.0) continue;
        if (!project(cam, fwd, lat, &u, &v)) continue;

        u = floor(u + 0.5);
        v = floor(v + 0.5);
        if (np > 0 && u == ux[np - 1] && v == vy[np - 1]) continue;
        ux[np] = u; vy[np] = v; np++;
    }

    if (np < 2) return 0;

    out[0].x0 = (float)ux[0];
    out[0].y0 = (float)vy[0];
    out[0].x1 = (float)ux[np - 1];
    out[0].y1 = (float)vy[np - 1];
    return 1;
}

/*
 * A point on the track -> the pixel the camera would report it at, rounded to the
 * grid. The exact inverse of what intersection.c does to get back out to the
 * ground, so the geometry cases can be written where the black lines really are.
 */
static void gproject(double fwd, double lat, float *u, float *v)
{
    double d = (PIXY_FOCAL_PX * CAM_HEIGHT_CM) / fwd;

    *v = (float)floor(d + CAM_HORIZON_ROW + 0.5);
    *u = (float)floor(CAM_CENTER_X - (lat * PIXY_FOCAL_PX / fwd) + 0.5);
}

/* Every crossing edge that might be in shot, nearest crossing first. */
static int render_all_bars(const Cam *cam, double px, double py, double pth,
                           TrkSegment *out, int maxOut)
{
    int nseg = 0, k, c;

    for (k = 0; k < g_nXing && nseg < maxOut; k++)
    {
        for (c = 0; c < 4 && nseg < maxOut; c++)
        {
            double sb   = (c < 2) ? -1.0 : 1.0;
            double side = ((c & 1) == 0) ? 1.0 : -1.0;

            nseg += render_bar(cam, px, py, pth, k, sb, side,
                               out + nseg, maxOut - nseg);
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
    int    isecSeen;   /* camera frames with a corner detected        */
    int    isecCross;  /* crossings the car committed to              */
    double isecFirstS; /* centre line distance of the first commit, cm */
    double isecPeakLat;/* worst lateral offset while crossing, cm     */
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

/*
 * A second generator, for laying out circuits.
 *
 * It has to be separate from the one above, and that is not fastidiousness. The
 * fault injector calls frand() once per camera frame whether or not it is armed,
 * so a change that makes the car drive differently changes how many times it is
 * called, which changes every circuit generated afterwards. Two firmware builds
 * would then be scored on two different sets of tracks, and the comparison would
 * be worthless without ever looking wrong.
 */
static unsigned g_genRng = 1u;

static double grand(void)
{
    g_genRng = (g_genRng * 1103515245u) + 12345u;
    return (double)((g_genRng >> 16) & 0x7FFFu) / 32768.0;
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

/* ------------------------------------------------------------------ */
/* Machine learning dataset                                            */
/*
 * One row per camera frame: the feature vector features.c builds from that
 * frame, and the label the circuit generator already knows the answer to.
 *
 * There is no bootstrapping problem here and it is worth being clear why. The
 * label is not a judgement made about a picture - it is read straight out of the
 * geometry this file used to draw the picture in the first place.
 * build_centerline() integrates th += g_step * curv, so the curvature at any arc
 * length is a stored input; xing_add() puts each crossing at a known centimetre
 * mark. Nobody classifies the first ones. The generator already knows.
 *
 * The one real trap is labelling something the frame cannot show. If the camera
 * can see eighty centimetres and the label says "corner", meaning a bend two
 * metres away, the model is being trained to predict from evidence that is not
 * in its input - so it finds a shortcut in the simulator instead, scores well
 * offline, and falls apart on the car. So the horizon is not a constant here: it
 * is measured from the rendered frame, every frame.
 */
/* ------------------------------------------------------------------ */

enum { ML_STRAIGHT = 0, ML_CORNER = 1, ML_INTERSECTION = 2 };

/*
 * Where a kink becomes a corner. race_config.h already draws this line for the
 * speed planner - "dead straight curv ~0.1, 90 cm radius corner curv ~1.5" - and
 * the label has to land the same way, or the classifier disagrees with driver.c
 * by construction and the two spend the lap arguing.
 */
#define ML_CORNER_RADIUS_CM   150.0

/* Past this the crossing is a couple of pixels at the top of the frame and the
 * evidence is too thin to call. Inside it, the mouth is genuinely in shot. */
#define ML_ISEC_MAX_CM        130.0
/* Still an intersection once the car is in it, until the back axle is clear. */
#define ML_ISEC_BEHIND_CM     25.0

/* Below this the frame shows a scrap of line and nothing that supports a label. */
#define ML_MIN_SIGHT_CM       35.0

static FILE     *g_mlFile  = NULL;
static uint16_t  g_mlTrack = 0;
static uint16_t  g_mlFrame = 0;
static long      g_mlRows  = 0;
static long      g_mlDropped = 0;
static long      g_mlClass[3] = { 0, 0, 0 };

/* The label for the pose the current frame describes. sightCm is how far ahead
 * that frame actually shows a line, so the window never outruns the evidence. */
static int ml_label(int senseIdx, double sightCm)
{
    double s         = (double)senseIdx * g_step;
    double isecAhead = (sightCm < ML_ISEC_MAX_CM) ? sightCm : ML_ISEC_MAX_CM;
    double sum       = 0.0;
    int    k, i, steps, cnt = 0;

    for (k = 0; k < g_nXing; k++)
    {
        double d      = g_xing[k].at - s;
        double behind = -((g_xing[k].w * 0.5) + ML_ISEC_BEHIND_CM);

        if (d > behind && d < isecAhead)
        {
            return ML_INTERSECTION;
        }
    }

    steps = (int)(sightCm / g_step);
    for (i = senseIdx; (i < senseIdx + steps) && ((i + 1) < g_cn); i++)
    {
        sum += fabs(g_cth[i + 1] - g_cth[i]) / g_step;   /* 1/cm */
        cnt++;
    }
    if (cnt == 0)
    {
        return ML_STRAIGHT;
    }

    return ((sum / (double)cnt) > (1.0 / ML_CORNER_RADIUS_CM)) ? ML_CORNER
                                                              : ML_STRAIGHT;
}

static void ml_emit(const TrkSegment *segs, uint8_t n, const TrackModel *tm,
                    int senseIdx)
{
    float         feat[FEAT_N];
    unsigned char hdr[5];
    double        sightCm;
    int           label;

    if (g_mlFile == NULL)
    {
        return;
    }

    /*
     * The centre line is finite and the car is quicker than the time budget
     * assumes, so it reaches the end and drives on into nothing. Everything past
     * here is a view of blank floor with a label read off the last centre line
     * index - fiction, and a lot of it. Stop a clear sight distance short.
     */
    if (senseIdx >= (g_cn - (int)(200.0 / g_step)))
    {
        return;
    }

    Features_Build(segs, n, tm, feat);

    /* FEAT_TOP_REACH is the farthest point on any up-track segment, scaled by
     * 200 cm on the way in. Undo that and it is the sight distance in cm. */
    sightCm = (double)feat[FEAT_TOP_REACH] * 200.0;

    /*
     * A frame with nothing usable in it is not a training example, it is a
     * missing one. The car is blind here - lost, mid packet drop, or pointed at
     * bare floor - and every label would be a statement about the world that the
     * input cannot support. Keeping them would teach the most common thing in the
     * set: "when you can see nothing, say straight". Which is how a classifier
     * learns to drive confidently into a junction it cannot see.
     */
    if (sightCm < ML_MIN_SIGHT_CM || tm->nValid == 0u)
    {
        g_mlDropped++;
        return;
    }

    label = ml_label(senseIdx, sightCm);

    hdr[0] = (unsigned char)(g_mlTrack & 0xFFu);
    hdr[1] = (unsigned char)((g_mlTrack >> 8) & 0xFFu);
    hdr[2] = (unsigned char)(g_mlFrame & 0xFFu);
    hdr[3] = (unsigned char)((g_mlFrame >> 8) & 0xFFu);
    hdr[4] = (unsigned char)label;

    (void)fwrite(hdr, 1u, sizeof(hdr), g_mlFile);
    (void)fwrite(feat, sizeof(float), (size_t)FEAT_N, g_mlFile);

    g_mlFrame++;
    g_mlRows++;
    g_mlClass[label]++;
}

/* ------------------------------------------------------------------ */
/* Scoring the classifier against the geometry it is meant to help      */
/* ------------------------------------------------------------------ */

static int  g_chkOn = 0;
static long g_chkCM[3][3];              /* true class x predicted class      */
static int  g_chkKind = 0;              /* 0 square on, 1 just past a corner */
static int  g_chkTrackId = 0;
static int  g_chkSawTrue, g_chkSawIsec, g_chkSawNet;
static int  g_chkTracks[2], g_chkFoundIsec[2], g_chkFoundNet[2], g_chkFoundBoth[2];

/*
 * The dangerous direction, counted separately.
 *
 * Missing a crossing means turning down the wrong road. Inventing one in the
 * middle of a real corner means driving straight on, at speed, into the black
 * line. Those are not the same mistake, and a report that only counts crossings
 * found is measuring the half of the problem that cannot hurt the car.
 *
 * A "false raise" is what the firmware would actually act on: CLS_MIN_HITS
 * frames in a row, each at or above CLS_MIN_PROB, on a stretch of track where
 * there is no crossing at all.
 */
static int  g_chkFalseRun;      /* consecutive confident wrong calls, this track */
static int  g_chkFalseTracks;   /* circuits with at least one such run           */
static int  g_chkFalseRaises;   /* runs in total                                  */
static int  g_chkFalseHere;
static int  g_chkIsecFalseRun, g_chkIsecFalseTracks, g_chkIsecFalseHere;

/*
 * Every frame's verdict, kept so the operating point can be swept afterwards
 * without re-driving the circuits. CLS_MIN_PROB and CLS_MIN_HITS are the two
 * numbers that decide how eager the classifier is, and they trade the same way
 * every detector does: catch more crossings, invent more of them. Which end of
 * that trade is right is not something a simulator can settle - it depends on
 * whether the track has crossings the geometry already handles - so the sweep
 * prints the curve and leaves the choice where it belongs.
 */
#define SW_MAX 400000
static float    *g_swProb;
static uint8_t  *g_swTruth;
static uint16_t *g_swTrack;
static long      g_swN;

static Classifier g_chkNet;

static void chk_frame(const TrkSegment *segs, uint8_t n, const DriveState *st,
                      int senseIdx)
{
    float  feat[FEAT_N];
    double sightCm;
    int    truth, pred;

    if (!g_chkOn)
    {
        return;
    }
    if (senseIdx >= (g_cn - (int)(200.0 / g_step)))
    {
        return;
    }

    Features_Build(segs, n, &st->track, feat);
    sightCm = (double)feat[FEAT_TOP_REACH] * 200.0;
    if (sightCm < ML_MIN_SIGHT_CM || st->track.nValid == 0u)
    {
        return;
    }

    truth = ml_label(senseIdx, sightCm);
    pred  = (int)Classifier_Step(&g_chkNet, segs, n, &st->track);

    g_chkCM[truth][pred]++;

    if ((g_swProb != NULL) && (g_swN < SW_MAX))
    {
        g_swProb[g_swN]  = g_chkNet.prob[CLS_INTERSECTION];
        g_swTruth[g_swN] = (uint8_t)truth;
        g_swTrack[g_swN] = (uint16_t)g_chkTrackId;
        g_swN++;
    }

    if (truth == ML_INTERSECTION)
    {
        g_chkSawTrue++;
        /* The geometry counts as having seen it if either the per frame
         * detector fired or the latch is actually running. */
        if (st->isec.seen || st->isec.crossing) g_chkSawIsec++;
        if (pred == ML_INTERSECTION)            g_chkSawNet++;

        g_chkFalseRun     = 0;
        g_chkIsecFalseRun = 0;
    }
    else
    {
        /* No crossing here. Would the firmware have raised one anyway? */
        if ((pred == ML_INTERSECTION) &&
            (g_chkNet.prob[CLS_INTERSECTION] >= CLS_MIN_PROB))
        {
            g_chkFalseRun++;
            if (g_chkFalseRun == (int)CLS_MIN_HITS)
            {
                g_chkFalseRaises++;
                g_chkFalseHere = 1;
            }
        }
        else
        {
            g_chkFalseRun = 0;
        }

        if (st->isec.seen)
        {
            g_chkIsecFalseRun++;
            if (g_chkIsecFalseRun == (int)ISEC_CONFIRM_FRAMES)
            {
                g_chkIsecFalseHere = 1;
            }
        }
        else
        {
            g_chkIsecFalseRun = 0;
        }
    }
}

static Result run(const TrackSeg *segs, int nsegs, const Cam *cam, double halfW,
                  double startLat, const char *name, int verbose, double maxTime)
{
    Car        car;
    Result     r;
    TrkSegment allSegs[PIXY_MAX_VECTORS];
    DriveCmd   cmd;
    double     t = 0.0, dt = 0.004; /* 250 Hz control loop */
    int        hint = 0;
    int        senseIdx = 0;        /* centre line index the latest frame describes */
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
    r.isecFirstS = -1.0;
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
            senseIdx = sidx;   /* the frame describes where the car WAS, not where it is */
            k = render_edge(cam, sx, sy, sth, sidx, +1.0, allSegs, 4);
            n = (uint8_t)k;
            k = render_edge(cam, sx, sy, sth, sidx, -1.0, allSegs + n,
                            PIXY_MAX_VECTORS - n);
            n = (uint8_t)(n + k);

            /* ...and every crossing edge that might be in shot with them. */
            k = render_all_bars(cam, sx, sy, sth, allSegs + n, PIXY_MAX_VECTORS - n);
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

            /* Post fault injection on purpose: the row records what the car
             * actually saw this frame, dropped packets and all. */
            ml_emit(allSegs, n, &st->track, senseIdx);
            chk_frame(allSegs, n, st, senseIdx);

            if (!st->track.haveTrack) r.lostFrames++;
            else if (!st->track.bothEdges) r.oneEdgeFrames++;
            if (st->line.chicane) r.chicaneFrames++;

            if (st->isec.seen) r.isecSeen++;
            if ((int)st->isec.count > r.isecCross)
            {
                if (r.isecFirstS < 0.0) r.isecFirstS = idx * g_step;
                r.isecCross = (int)st->isec.count;
            }

            if ((g_isecTrace == 1 && st->isec.seen) ||
                (g_isecTrace > 1 && g_nXing > 0 &&
                 fabs((idx * g_step) - g_xing[0].at) < 150.0))
            {
                int q;
                printf("  t=%5.2f s=%6.1f  L=%d R=%d gap=%.0f..%.0f (%.0f cm) "
                       "edges=%d slope=%+.2f steer=%+5.1f auth=%.2f %s\n",
                       t, idx * g_step,
                       st->isec.sawLeft, st->isec.sawRight,
                       st->isec.gapStartCm, st->isec.gapEndCm, st->isec.gapCm,
                       st->isec.nEdges, st->isec.slope, st->isec.steer,
                       st->isec.authority, st->isec.crossing ? "CROSSING" : "");
                printf("        track rows=%d both=%d hN=%+.2f hF=%+.2f "
                       "tgt=%.1f steer=%+.1f v=%.0f lat=%+.1f\n",
                       st->track.nValid, st->track.bothEdges,
                       st->track.headNear, st->track.headFar,
                       st->line.targetX, cmd.steer, car.v,
                       lateral_offset(car.x, car.y, idx));

                for (q = 0; q < (int)n; q++)
                    printf("        seg%d (%2.0f,%2.0f)->(%2.0f,%2.0f)\n", q,
                           allSegs[q].x0, allSegs[q].y0,
                           allSegs[q].x1, allSegs[q].y1);
            }
        }

        if (Driver_State()->isec.crossing)
        {
            double al = fabs(lateral_offset(car.x, car.y, idx));
            if (al > r.isecPeakLat) r.isecPeakLat = al;
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

/* ------------------------------------------------------------------ */
/* Procedural circuits for the dataset                                  */
/* ------------------------------------------------------------------ */

#define GEN_MAX_SEG 32

/*
 * Lays out one random circuit and the crossings on it, and picks a camera
 * mounting to view it through.
 *
 * kind 0  a general mixture: straights of every length, bends from 50 cm to
 *         270 cm radius, chicanes, crossings dropped anywhere along the lap -
 *         and, three times in ten, no crossing at all, because a classifier
 *         trained without honest negatives learns to see junctions everywhere.
 *
 * kind 1  the case that motivates all of this. The crossing sits a few
 *         centimetres past the exit of a real corner, so the car is still
 *         rotating when the junction enters the frame. It never sees the mouth
 *         square on; it sees one corner of it at an angle, which is the picture
 *         that makes intersection.c report "crossing just after a bend, no latch
 *         needed" and drive on.
 *
 * The camera mounting is drawn continuously from the same envelope -sweep steps
 * through discretely, and g_maxChunks is randomised too: at 1 the renderer
 * chords a whole curve into a single straight vector, which is what the real
 * Pixy2 does and what the README calls the single worst thing the camera can do.
 * A model that has only ever seen clean three-chunk curves has not met the
 * camera it will be racing behind.
 */
static int gen_track(TrackSeg *t, int kind, double *halfW, Cam *cam,
                     double *startLat)
{
    int    n = 0, i, nFeat;
    double s = 0.0;
    double xAt = -1.0;

    *halfW = 20.0 + grand() * 10.0;

    /*
     * Draw a mounting, and keep drawing until it is one a car could actually race
     * behind - the same test -envelope applies, continuous instead of on a grid.
     *
     * This is not tidiness. A badly aimed camera puts the car off the track within
     * a few seconds, and every frame after that is a wheel-in-the-grass view with
     * nothing to classify. Training on those teaches the model what a crashed car
     * sees. The mounting still varies over the whole sane range, which is the part
     * worth being robust to.
     */
    for (i = 0; i < 40; i++)
    {
        double dNear, dFar, halfViewMid;

        cam->f     = 55.0 + grand() * 25.0;
        cam->h     = 12.0 + grand() * 14.0;
        cam->horiz = -10.0 + grand() * 7.0;

        dNear       = cam->f * cam->h / (51.0 - cam->horiz);
        dFar        = cam->f * cam->h / (0.0 - cam->horiz);
        halfViewMid = (0.5 * (PIXY_LINE_W - 1)) * cam->h / (26.0 - cam->horiz);

        if (dNear >= 15.0 && dNear <= 35.0 && dFar >= 80.0 && dFar <= 250.0 &&
            halfViewMid >= *halfW)
        {
            break;
        }
    }

    *startLat = (grand() - 0.5) * (*halfW) * 0.8;

    {
        double roll = grand();
        g_maxChunks = (roll < 0.40) ? 1 : ((roll < 0.70) ? 2 : 3);
    }
    g_dropRate = (grand() < 0.5) ? 0.0 : (grand() * 0.06);

    /* An opening straight, so the car is settled before anything happens. */
    t[n].curv = 0.0;
    t[n].len  = 100.0 + grand() * 110.0;
    s += t[n].len;
    n++;

    nFeat = 4 + (int)(grand() * 6.0);

    for (i = 0; (i < nFeat) && (n < GEN_MAX_SEG - 3); i++)
    {
        double roll = grand();

        if (kind == 1 && i == 1)
        {
            double R     = 55.0 + grand() * 70.0;
            double sweep = (M_PI / 3.0) + grand() * (M_PI * 0.7);
            double sgn   = (grand() < 0.5) ? 1.0 : -1.0;

            t[n].curv = sgn / R;
            t[n].len  = R * sweep;
            s += t[n].len;
            n++;

            /* 5 to 70 cm past the exit. Near the bottom of that range the car is
             * still turning as the junction appears; near the top it has just
             * straightened and the mouth is skewed rather than square. */
            xAt = s + 5.0 + grand() * 65.0;

            t[n].curv = 0.0;
            t[n].len  = 230.0;
            s += t[n].len;
            n++;
            continue;
        }

        if (roll < 0.26)
        {
            t[n].curv = 0.0;
            t[n].len  = 50.0 + grand() * 190.0;
            s += t[n].len;
            n++;
        }
        else if (roll < 0.78)
        {
            double R     = 50.0 + grand() * 220.0;
            double sweep = 0.4 + grand() * (M_PI * 0.9);
            double sgn   = (grand() < 0.5) ? 1.0 : -1.0;

            t[n].curv = sgn / R;
            t[n].len  = R * sweep;
            s += t[n].len;
            n++;
        }
        else
        {
            double R   = 90.0 + grand() * 140.0;
            double sw  = 0.25 + grand() * 0.45;
            double sgn = (grand() < 0.5) ? 1.0 : -1.0;

            t[n].curv = sgn / R;
            t[n].len  = R * sw;
            s += t[n].len;
            n++;
            t[n].curv = -sgn / R;
            t[n].len  = R * sw;
            s += t[n].len;
            n++;
        }
    }

    t[n].curv = 0.0;
    t[n].len  = 160.0;
    s += t[n].len;
    n++;

    xing_clear();
    if (kind == 1)
    {
        xing_add(xAt, 35.0 + grand() * 20.0, 15.0 + grand() * 30.0,
                 (grand() < 0.35) ? 0 : 1);
    }
    else
    {
        double roll = grand();

        if (roll >= 0.30)
        {
            double span = (s - 320.0 > 160.0) ? (s - 320.0) : 160.0;
            int    k, nx = (roll < 0.85) ? 1 : 2;

            for (k = 0; k < nx; k++)
            {
                xing_add(160.0 + grand() * span,
                         35.0 + grand() * 20.0,
                         15.0 + grand() * 30.0,
                         (grand() < 0.30) ? 0 : 1);
            }
        }
    }

    return n;
}

/* Generates the whole dataset and writes it out. */
static int ml_dataset(const char *path, int nTracks, int nOblique, unsigned seed)
{
    TrackSeg t[GEN_MAX_SEG];
    Cam      cam;
    unsigned hdr[8];
    double   halfW, startLat, lenCm;
    int      i, nsegs, kind;
    long     rowsAt;

    g_mlFile = fopen(path, "wb");
    if (g_mlFile == NULL)
    {
        fprintf(stderr, "cannot open %s for writing\n", path);
        return 1;
    }

    g_genRng = seed;

    hdr[0] = 0x314C4D54u;         /* "TML1" */
    hdr[1] = 1u;                  /* version                      */
    hdr[2] = (unsigned)FEAT_N;    /* floats per row               */
    hdr[3] = (unsigned)FHIST_N;   /* scalars classifier.c stacks  */
    hdr[4] = 0u;                  /* row count, filled in at the end */
    hdr[5] = (unsigned)nTracks;
    hdr[6] = (unsigned)nOblique;
    hdr[7] = seed;
    (void)fwrite(hdr, sizeof(unsigned), 8u, g_mlFile);

    printf("generating %d tracks (%d with the crossing just past a corner exit)\n",
           nTracks, nOblique);

    for (i = 0; i < nTracks; i++)
    {
        /* Spread the oblique circuits evenly through the run rather than putting
         * them in a block: any prefix of the file is then a representative
         * sample, so a short generation is still a usable experiment. Fires
         * exactly nOblique times in nTracks. */
        kind = ((((i + 1) * nOblique) / nTracks) != ((i * nOblique) / nTracks))
             ? 1 : 0;

        nsegs = gen_track(t, kind, &halfW, &cam, &startLat);

        lenCm = 0.0;
        {
            int q;
            for (q = 0; q < nsegs; q++) lenCm += t[q].len;
        }

        g_mlTrack = (uint16_t)i;
        g_mlFrame = 0u;

        (void)run(t, nsegs, &cam, halfW, startLat, "ml", 0,
                  (lenCm / 150.0) + 2.0);

        if (((i + 1) % 100) == 0)
        {
            printf("  %4d/%d tracks   %ld rows\n", i + 1, nTracks, g_mlRows);
            (void)fflush(stdout);
        }
    }

    rowsAt = (long)(sizeof(unsigned) * 4u);
    if (fseek(g_mlFile, rowsAt, SEEK_SET) == 0)
    {
        unsigned rows = (unsigned)g_mlRows;
        (void)fwrite(&rows, sizeof(unsigned), 1u, g_mlFile);
    }
    (void)fclose(g_mlFile);
    g_mlFile = NULL;

    printf("\n%ld rows, %d features each -> %s\n", g_mlRows, FEAT_N, path);
    printf("  (%ld frames dropped: camera had nothing to answer from)\n", g_mlDropped);
    printf("  straight     %8ld  %5.1f%%\n", g_mlClass[ML_STRAIGHT],
           100.0 * (double)g_mlClass[ML_STRAIGHT] / (double)g_mlRows);
    printf("  corner       %8ld  %5.1f%%\n", g_mlClass[ML_CORNER],
           100.0 * (double)g_mlClass[ML_CORNER] / (double)g_mlRows);
    printf("  intersection %8ld  %5.1f%%\n", g_mlClass[ML_INTERSECTION],
           100.0 * (double)g_mlClass[ML_INTERSECTION] / (double)g_mlRows);

    return 0;
}

/*
 * Runs the trained classifier and intersection.c side by side over circuits
 * neither has seen, and reports what each of them found.
 *
 * The interesting column is the second one. A crossing approached square on is
 * the case the geometry was designed for and is good at. A crossing just past
 * the exit of a corner is the case its own test suite records as "crossing just
 * after a bend ... no latch needed", which is a polite way of saying it drove
 * past. That is the row worth reading.
 */
/*
 * Replays the recorded per frame probabilities at a range of settings for
 * CLS_MIN_PROB and CLS_MIN_HITS, and reports what each would have done.
 *
 * "found" is a crossing that got the required run of confident frames while one
 * was genuinely in shot. "false" is a circuit where the same run happened where
 * there was no crossing at all - which, on the car, is the steering being handed
 * to a junction that is not there.
 */
static void ml_sweep(int nTracks)
{
    static const float PROBS[] = { 0.50f, 0.70f, 0.80f, 0.90f, 0.95f, 0.98f };
    static const int   HITS[]  = { 3, 5, 8 };
    int   pi, hi;

    if ((g_swProb == NULL) || (g_swN == 0))
    {
        return;
    }

    printf("\noperating points - pick one, then set CLS_MIN_PROB and CLS_MIN_HITS\n");
    printf("%8s %6s %14s %16s\n", "prob", "hits", "crossings found",
           "circuits w/ false");

    for (hi = 0; hi < (int)(sizeof(HITS) / sizeof(HITS[0])); hi++)
    {
        for (pi = 0; pi < (int)(sizeof(PROBS) / sizeof(PROBS[0])); pi++)
        {
            float p    = PROBS[pi];
            int   need = HITS[hi];
            long  k;
            int   run = 0, falseRun = 0;
            int   found = 0, total = 0, falseTracks = 0;
            int   sawTrue = 0, hitHere = 0, falseHere = 0;
            uint16_t cur = g_swTrack[0];

            for (k = 0; k <= g_swN; k++)
            {
                bool endOfTrack = (k == g_swN) || (g_swTrack[k] != cur);

                if (endOfTrack)
                {
                    if (sawTrue > 0) { total++; if (hitHere) found++; }
                    if (falseHere) falseTracks++;
                    run = 0; falseRun = 0; sawTrue = 0; hitHere = 0; falseHere = 0;
                    if (k == g_swN) break;
                    cur = g_swTrack[k];
                }

                if (g_swTruth[k] == (uint8_t)ML_INTERSECTION)
                {
                    sawTrue++;
                    falseRun = 0;
                    run = (g_swProb[k] >= p) ? (run + 1) : 0;
                    if (run >= need) hitHere = 1;
                }
                else
                {
                    run = 0;
                    falseRun = (g_swProb[k] >= p) ? (falseRun + 1) : 0;
                    if (falseRun >= need) falseHere = 1;
                }
            }

            printf("%8.2f %6d %9d %4.0f%% %11d %4.0f%%\n",
                   (double)p, need,
                   found, total ? (100.0 * found / total) : 0.0,
                   falseTracks, 100.0 * falseTracks / nTracks);
        }
        printf("\n");
    }
}

static int ml_check(int nTracks, int nOblique, unsigned seed)
{
    TrackSeg t[GEN_MAX_SEG];
    Cam      cam;
    double   halfW, startLat, lenCm;
    int      i, q, nsegs, kind;
    long     tot = 0, right = 0;

    g_genRng = seed;
    g_chkOn  = 1;
    (void)memset(g_chkCM, 0, sizeof(g_chkCM));
    (void)memset(g_chkTracks, 0, sizeof(g_chkTracks));
    (void)memset(g_chkFoundIsec, 0, sizeof(g_chkFoundIsec));
    (void)memset(g_chkFoundNet, 0, sizeof(g_chkFoundNet));
    (void)memset(g_chkFoundBoth, 0, sizeof(g_chkFoundBoth));
    g_chkFalseTracks = 0;
    g_chkFalseRaises = 0;
    g_chkIsecFalseTracks = 0;

    g_swN     = 0;
    g_swProb  = (float *)malloc(sizeof(float) * SW_MAX);
    g_swTruth = (uint8_t *)malloc(SW_MAX);
    g_swTrack = (uint16_t *)malloc(sizeof(uint16_t) * SW_MAX);
    if ((g_swProb == NULL) || (g_swTruth == NULL) || (g_swTrack == NULL))
    {
        free(g_swProb); free(g_swTruth); free(g_swTrack);
        g_swProb = NULL; g_swTruth = NULL; g_swTrack = NULL;
    }

    printf("=== classifier vs intersection.c, on %d unseen circuits ===\n\n",
           nTracks);

    for (i = 0; i < nTracks; i++)
    {
        kind = ((((i + 1) * nOblique) / nTracks) != ((i * nOblique) / nTracks))
             ? 1 : 0;

        nsegs = gen_track(t, kind, &halfW, &cam, &startLat);

        lenCm = 0.0;
        for (q = 0; q < nsegs; q++) lenCm += t[q].len;

        g_chkKind    = kind;
        g_chkTrackId = i;
        g_chkSawTrue = 0;
        g_chkSawIsec = 0;
        g_chkSawNet  = 0;
        g_chkFalseRun = 0;
        g_chkFalseHere = 0;
        g_chkIsecFalseRun = 0;
        g_chkIsecFalseHere = 0;
        Classifier_Init(&g_chkNet);

        (void)run(t, nsegs, &cam, halfW, startLat, "chk", 0,
                  (lenCm / 150.0) + 2.0);

        /* Only circuits where a crossing was actually in shot at some point can
         * say anything about whether it was found. */
        if (g_chkFalseHere)     g_chkFalseTracks++;
        if (g_chkIsecFalseHere) g_chkIsecFalseTracks++;

        if (g_chkSawTrue > 0)
        {
            g_chkTracks[kind]++;
            /* Three frames, not one: a single frame agreeing is a coin landing
             * the right way up, and the latch in intersection.c needs
             * confirmation before it commits too. */
            if (g_chkSawIsec >= 3) g_chkFoundIsec[kind]++;
            if (g_chkSawNet  >= 3) g_chkFoundNet[kind]++;
            if (g_chkSawIsec >= 3 && g_chkSawNet >= 3) g_chkFoundBoth[kind]++;
        }
    }

    g_chkOn = 0;

    printf("per frame, true class down, predicted across\n");
    printf("%14s %12s %12s %12s %10s\n", "", "straight", "corner",
           "intersection", "recall");
    for (i = 0; i < 3; i++)
    {
        long rowSum = g_chkCM[i][0] + g_chkCM[i][1] + g_chkCM[i][2];

        printf("%14s %12ld %12ld %12ld %9.1f%%\n",
               Classifier_Name((ClassId)i),
               g_chkCM[i][0], g_chkCM[i][1], g_chkCM[i][2],
               rowSum ? (100.0 * (double)g_chkCM[i][i] / (double)rowSum) : 0.0);
        tot   += rowSum;
        right += g_chkCM[i][i];
    }
    printf("%14s", "precision");
    for (i = 0; i < 3; i++)
    {
        long colSum = g_chkCM[0][i] + g_chkCM[1][i] + g_chkCM[2][i];
        printf(" %11.1f%%", colSum ? (100.0 * (double)g_chkCM[i][i] / (double)colSum)
                                   : 0.0);
    }
    printf("\n\n%ld frames, %.1f%% correct overall\n\n",
           tot, tot ? (100.0 * (double)right / (double)tot) : 0.0);

    printf("crossings found, by how the car arrived at them\n");
    printf("%-26s %9s %13s %11s %9s\n", "approach", "crossings",
           "intersection.c", "classifier", "both");
    for (i = 0; i < 2; i++)
    {
        const char *nm = (i == 0) ? "square on" : "just past a corner exit";

        if (g_chkTracks[i] == 0) continue;

        printf("%-26s %9d %8d %4.0f%% %6d %4.0f%% %9d\n", nm, g_chkTracks[i],
               g_chkFoundIsec[i], 100.0 * g_chkFoundIsec[i] / g_chkTracks[i],
               g_chkFoundNet[i],  100.0 * g_chkFoundNet[i]  / g_chkTracks[i],
               g_chkFoundBoth[i]);
    }

    printf("\ncrossings raised where there was none - the mistake that hurts\n");
    printf("  classifier      %3d of %d circuits (%.0f%%), %d separate raises\n",
           g_chkFalseTracks, nTracks, 100.0 * g_chkFalseTracks / nTracks,
           g_chkFalseRaises);
    printf("  intersection.c  %3d of %d circuits (%.0f%%)\n",
           g_chkIsecFalseTracks, nTracks,
           100.0 * g_chkIsecFalseTracks / nTracks);

    ml_sweep(nTracks);
    return 0;
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

    /* -mlcheck [tracks] [oblique] [seed]
     * Scores the trained classifier against intersection.c on unseen circuits. */
    if (argc > 1 && strcmp(argv[1], "-mlcheck") == 0)
    {
        return ml_check((argc > 2) ? atoi(argv[2]) : 250,
                        (argc > 3) ? atoi(argv[3]) : 50,
                        (argc > 4) ? (unsigned)strtoul(argv[4], NULL, 10) : 90210u);
    }

    /* -mldata <file> [tracks] [oblique] [seed]
     * Writes one row per camera frame for the classifier trainer. */
    if (argc > 2 && strcmp(argv[1], "-mldata") == 0)
    {
        return ml_dataset(argv[2],
                          (argc > 3) ? atoi(argv[3]) : 1000,
                          (argc > 4) ? atoi(argv[4]) : 200,
                          (argc > 5) ? (unsigned)strtoul(argv[5], NULL, 10) : 1u);
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

    if (argc > 2 && strcmp(argv[1], "-frame") == 0)
    {
        /* ---------------------------------------------------------------
         * One frame in, one answer out: what would the car do with this?
         *
         * Vectors are given the way the Pixy2 reports them, on the 79 x 52
         * grid, as x0,y0,x1,y1 - so a screenshot of the line tracker view can
         * be measured off with a ruler and fed straight in.
         *
         *   ./track_sim -frame 1,51,38,8  38,8,7,2  62,51,75,21
         *
         * It prints where each vector really is on the track, what the corridor
         * model makes of the frame, whether the crossing detector sees anything,
         * and what the servo would be asked for.
         * -------------------------------------------------------------*/
        TrkSegment  sg[PIXY_MAX_VECTORS];
        uint8_t     n = 0;
        Intersection ix;
        DriveCmd    cmd;
        const DriveState *st;
        int         i;

        for (i = 2; (i < argc) && (n < PIXY_MAX_VECTORS); i++)
        {
            double a, b, c, d;

            if (sscanf(argv[i], "%lf,%lf,%lf,%lf", &a, &b, &c, &d) != 4) continue;
            sg[n].x0 = (float)a; sg[n].y0 = (float)b;
            sg[n].x1 = (float)c; sg[n].y1 = (float)d;
            n++;
        }
        if (n == 0)
        {
            printf("no vectors given\n");
            return 1;
        }

        /* Settle the width model on this frame first, so the mounting reported
         * below is the learned one the detector will use, not the fallback. */
        Driver_Init();
        Intersection_Init(&ix);
        for (i = 0; i < 250; i++)
        {
            Driver_Step(true, sg, n, 0.016f, &cmd);
        }

        printf("=== what the car does with this frame ===\n");
        printf("camera, learned from the width the car measures: focal %.0f px, "
               "height %.0f cm, horizon row %.0f\n",
               (double)PIXY_FOCAL_PX, (double)Track_CamHeightCm(),
               (double)Track_HorizonRow());
        printf("configured fallbacks, used only until it settles: height %.0f, "
               "horizon %.0f\n\n", (double)CAM_HEIGHT_CM, (double)CAM_HORIZON_ROW);

        printf("%-4s %-18s %-26s %s\n", "vec", "image", "on the track (cm)", "runs");
        for (i = 0; i < (int)n; i++)
        {
            double hz = Track_HorizonRow();
            double hh = Track_CamHeightCm();
            double d0 = sg[i].y0 - hz;
            double d1 = sg[i].y1 - hz;
            double f0 = (d0 > 0.5) ? (PIXY_FOCAL_PX * hh / d0) : 0.0;
            double f1 = (d1 > 0.5) ? (PIXY_FOCAL_PX * hh / d1) : 0.0;
            double l0 = (CAM_CENTER_X - sg[i].x0) * f0 / PIXY_FOCAL_PX;
            double l1 = (CAM_CENTER_X - sg[i].x1) * f1 / PIXY_FOCAL_PX;

            printf("%-4d (%2.0f,%2.0f)->(%2.0f,%2.0f)  %5.0f cm ahead %+5.0f left ->"
                   " %4.0f %+5.0f  %s\n", i,
                   sg[i].x0, sg[i].y0, sg[i].x1, sg[i].y1, f0, l0, f1, l1,
                   (fabs(f1 - f0) > fabs(l1 - l0)) ? "up the track" : "across it");
        }

        Intersection_Update(sg, n, &ix);
        st = Driver_State();

        printf("\n-- the corridor --\n");
        printf("  usable rows      %d of %d%s\n", st->track.nValid, TRK_ROWS,
               st->track.haveTrack ? "" : "   (not enough to drive on)");
        printf("  both lines seen  %s\n", st->track.bothEdges ? "yes" : "no");
        if (st->track.nValid > 0)
        {
            printf("  width at the bumper  %.0f px, centre %.0f  (39 = straight ahead)\n",
                   st->track.width[0], st->track.center[0]);
            printf("  heading near %+.2f  far %+.2f  (over ~1.5 is a real corner)\n",
                   st->track.headNear, st->track.headFar);
        }

        printf("\n-- the crossing detector --\n");
        printf("  lines running up the track: %d\n", ix.nEdges);
        if (ix.seen)
        {
            printf("  HOLE FOUND on the %s: the lines stop %.0f cm ahead and start\n",
                   (ix.sawLeft && ix.sawRight) ? "both sides"
                                               : (ix.sawLeft ? "left" : "right"),
                   ix.gapStartCm);
            printf("  again %.0f cm ahead - a %.0f cm gap, which is one track width.\n",
                   ix.gapEndCm, ix.gapCm);
            printf("  it would commit at %.0f cm and drive straight over.\n",
                   (double)ISEC_COMMIT_CM);
        }
        else
        {
            printf("  no crossing recognised from this frame alone.\n");
        }

        printf("\n-- what the servo is asked for --\n");
        printf("  aim point   column %.1f  (39 = straight ahead)\n", st->line.targetX);
        printf("  steer       %+.1f  (%s)\n", cmd.steer,
               (cmd.steer > 3.0f) ? "right" : ((cmd.steer < -3.0f) ? "left" : "straight"));
        printf("  speed       %.0f\n", cmd.speed);
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "-many") == 0)
    {
        /* ---------------------------------------------------------------
         * A few hundred circuits nobody chose.
         *
         * Twelve hand written layouts are twelve opinions about what a track
         * looks like, and a tune can be fitted to them without anyone noticing.
         * These are generated from a seed: random corner radii and directions,
         * random straight lengths, random track width, the car put down at a
         * random offset, and a four road crossing dropped onto a straight
         * wherever there is room for one.
         *
         *     ./track_sim -many            250 circuits, seed 1
         *     ./track_sim -many 500 7      500 circuits, seed 7
         *     ./track_sim -many 250 1 -q   totals only
         * -------------------------------------------------------------*/
        int nWant = (argc > 2) ? atoi(argv[2]) : 250;
        int seed  = (argc > 3) ? atoi(argv[3]) : 1;
        int quiet = (argc > 4 && strcmp(argv[4], "-q") == 0);
        int i;

        int    okNo = 0, okX = 0, latched = 0, wanted = 0;
        double timeNo = 0.0, timeX = 0.0;
        double worstNo = 1e9, worstX = 1e9;

        if (nWant < 1) nWant = 1;
        g_genRng = ((unsigned)seed * 2654435761u) + 12345u;

        if (!quiet)
        {
            printf("=== %d generated circuits, seed %d ===\n", nWant, seed);
            printf("random corners, straights, track width and starting offset;\n");
            printf("each driven twice, with a four road crossing and without.\n\n");
            printf("%5s %-10s %7s %7s %6s  %s\n",
                   "n", "circuit", "no x", "with x", "cross", "verdict");
        }

        for (i = 0; i < nWant; i++)
        {
            TrackSeg t[14];
            double   straightAt[14];
            double   straightLen[14];
            int      nStr = 0;
            int      n = 0;
            double   at = 0.0;
            double   halfW, lat;
            char     name[64];
            Result   rn, rx;
            int      nseg, k, bad;
            double   xAt = -1.0;

            nseg  = 5 + (int)(grand() * 8.0);
            if (nseg > 13) nseg = 13;
            halfW = 17.5 + (grand() * 12.5);

            for (k = 0; (k < nseg) && (n < 13); k++)
            {
                if ((k & 1) == 0)
                {
                    double len = 60.0 + (grand() * 260.0);

                    if (k == 0) len += 120.0;
                    t[n].curv = 0.0;
                    t[n].len  = len;
                    straightAt[nStr]  = at;
                    straightLen[nStr] = len;
                    nStr++;
                    at += len;
                    n++;
                }
                else
                {
                    /* Radius 55 to 150, 30 to 180 degrees, either way. The car
                     * turns better left than right, so a tight right is given a
                     * little more room - that is a property of the steering stop,
                     * not of the track, and testing past it measures nothing. */
                    double r   = 55.0 + (grand() * 95.0);
                    double ang = (30.0 + (grand() * 150.0)) * M_PI / 180.0;
                    double dir = (grand() < 0.5) ? 1.0 : -1.0;

                    if ((dir < 0.0) && (r < 75.0)) r = 75.0;
                    t[n].curv = dir / r;
                    t[n].len  = r * ang;
                    at += t[n].len;
                    n++;
                }
            }

            if (n < 14)
            {
                t[n].curv = 0.0;
                t[n].len  = 200.0;
                straightAt[nStr]  = at;
                straightLen[nStr] = 200.0;
                nStr++;
                at += 200.0;
                n++;
            }

            lat = ((grand() * 1.5) - 0.75) * (halfW - 7.0);
            sprintf(name, "gen%d", i);

            xing_clear();
            rn = run(t, n, &cam, halfW, lat, name, 0, 45.0);
            if ((rn.excursions == 0) && rn.finished)
            {
                okNo++;
                timeNo += rn.lapTime;
            }
            if (rn.minClear < worstNo) worstNo = rn.minClear;

            /* A crossing goes on a straight long enough to hold one, far enough
             * in that the car is up to speed, and not so late that the run ends
             * before it gets there. */
            xing_clear();
            for (k = 0; k < nStr; k++)
            {
                if ((straightLen[k] >= 110.0) && (straightAt[k] > 150.0) &&
                    ((straightAt[k] + straightLen[k]) < (at - 220.0)))
                {
                    xAt = straightAt[k] + 32.5 +
                          (grand() * (straightLen[k] - 110.0));
                    break;
                }
            }
            if (xAt > 0.0)
            {
                xing_add(xAt, 45.0, 40.0, 1);
                wanted++;
            }

            rx  = run(t, n, &cam, halfW, lat, name, 0, 45.0);
            bad = (rx.excursions > 0) || !rx.finished;
            if (!bad)
            {
                okX++;
                timeX += rx.lapTime;
            }
            latched += rx.isecCross;
            if (rx.minClear < worstX) worstX = rx.minClear;

            if (!quiet)
            {
                int okn = (rn.excursions == 0) && rn.finished;

                printf("%5d %-10s %7s %7s %2d/%-2d  %s\n", i, name,
                       okn ? "clean" : "OFF", bad ? "OFF" : "clean",
                       rx.isecCross, (xAt > 0.0) ? 1 : 0,
                       (bad && okn) ? "*** the crossing lost it ***"
                                    : (bad ? "off without one too" : "ok"));
            }
        }

        xing_clear();
        printf("\nMANY n=%d seed=%d noX=%d withX=%d latched=%d/%d "
               "tNo=%.1f tX=%.1f clrNo=%+.2f clrX=%+.2f\n",
               nWant, seed, okNo, okX, latched, wanted,
               timeNo, timeX, worstNo, worstX);
        printf("  without a crossing : %5.1f%% of circuits driven clean\n",
               100.0 * okNo / nWant);
        printf("  with one           : %5.1f%% clean, %5.1f%% of crossings recognised\n",
               100.0 * okX / nWant,
               wanted ? (100.0 * latched / wanted) : 0.0);
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "-tracks") == 0)
    {
        /* ---------------------------------------------------------------
         * Every kind of track, with a four road crossing dropped into it
         * wherever a real one could go.
         *
         * A crossing is a straight tile, so it never sits on a curve - but the
         * straight it sits on can be very short, and that is the case that
         * decides a race. The placements below are quoted as the gap between
         * the end of the corner and the near edge of the crossing: 0 cm means
         * the black lines stop the instant the corner does, and the car is
         * still unwinding when it arrives.
         *
         * Every layout is run once with the crossing and once without, so what
         * the crossing costs is separated from what the corner costs.
         *
         *     ./track_sim -tracks         all of it
         *     ./track_sim -tracks 4       layout 4 on its own
         *     ./track_sim -tracks 4 -vv   ...with a frame by frame trace
         * -------------------------------------------------------------*/
        struct XCase { const char *where; double at; double second; };
        struct Layout
        {
            const char     *name;
            const TrackSeg *segs;
            int             n;
            double          halfW;
            double          startLat; /* where on the track the car is put down */
            double          maxT;
            struct XCase    x[6];
        };

        /* Corners all start 200 cm in. Arc lengths, so the placements read:
         * sweeper 141, medium 110, hairpin 173, S 84+84, decreasing 94+51,
         * increasing 47+86, double apex 73 + 60 straight + 73. */
        static const TrackSeg L_straight[] = {
            {0.0, 800.0},
        };
        static const TrackSeg L_sweeper[] = {
            {0.0, 200.0}, {1.0 / 90.0, 90.0 * (M_PI / 2)}, {0.0, 450.0},
        };
        static const TrackSeg L_medium[] = {
            {0.0, 200.0}, {1.0 / 70.0, 70.0 * (M_PI / 2)}, {0.0, 450.0},
        };
        static const TrackSeg L_hairpin[] = {
            {0.0, 200.0}, {1.0 / 55.0, 55.0 * M_PI}, {0.0, 450.0},
        };
        static const TrackSeg L_hairpinR[] = {
            {0.0, 200.0}, {-1.0 / 75.0, 75.0 * M_PI}, {0.0, 450.0},
        };
        static const TrackSeg L_sbend[] = {
            {0.0, 200.0}, {1.0 / 80.0, 80.0 * (M_PI / 3)},
            {-1.0 / 80.0, 80.0 * (M_PI / 3)}, {0.0, 450.0},
        };
        static const TrackSeg L_decreasing[] = {
            {0.0, 200.0}, {1.0 / 120.0, 120.0 * (M_PI / 4)},
            {1.0 / 65.0, 65.0 * (M_PI / 4)}, {0.0, 450.0},
        };
        static const TrackSeg L_increasing[] = {
            {0.0, 200.0}, {1.0 / 60.0, 60.0 * (M_PI / 4)},
            {1.0 / 110.0, 110.0 * (M_PI / 4)}, {0.0, 450.0},
        };
        static const TrackSeg L_doubleApex[] = {
            {0.0, 200.0}, {1.0 / 70.0, 70.0 * (M_PI / 3)}, {0.0, 70.0},
            {1.0 / 70.0, 70.0 * (M_PI / 3)}, {0.0, 450.0},
        };
        static const TrackSeg L_mixed[] = {
            {0.0, 260.0},
            {1.0 / 60.0, 60.0 * (M_PI / 2)}, {0.0, 120.0},
            {-1.0 / 85.0, 85.0 * (M_PI / 2)}, {0.0, 80.0},
            {-1.0 / 85.0, 85.0 * (M_PI / 2)}, {0.0, 200.0},
            {1.0 / 80.0, 80.0 * M_PI}, {0.0, 140.0},
            {1.0 / 55.0, 55.0 * (M_PI / 2)}, {0.0, 150.0},
        };

        static const struct Layout lay[] = {
            { "long straight", L_straight, 1, 22.5, 0.0, 40.0,
              { {"middle of it",        300.0, 0.0},
                {"two, 1.6 m apart",    300.0, 460.0},
                {"two, 1.0 m apart",    300.0, 400.0},
                {NULL, 0.0, 0.0} } },

            { "fast sweeper R90", L_sweeper, 3, 22.5, 0.0, 45.0,
              { {"25 cm before turn in", 152.5, 0.0},
                {"right at turn in",     177.5, 0.0},
                {"right at the exit",    363.9, 0.0},
                {"25 cm after exit",     388.9, 0.0},
                {"75 cm after exit",     438.9, 0.0},
                {NULL, 0.0, 0.0} } },

            { "medium corner R70", L_medium, 3, 22.5, 0.0, 45.0,
              { {"right at turn in",     177.5, 0.0},
                {"right at the exit",    332.5, 0.0},
                {"25 cm after exit",     357.5, 0.0},
                {NULL, 0.0, 0.0} } },

            { "hairpin R55 left", L_hairpin, 3, 22.5, 0.0, 50.0,
              { {"right at turn in",     177.5, 0.0},
                {"right at the exit",    395.3, 0.0},
                {"25 cm after exit",     420.3, 0.0},
                {"75 cm after exit",     470.3, 0.0},
                {NULL, 0.0, 0.0} } },

            { "hairpin R75 right", L_hairpinR, 3, 22.5, 0.0, 50.0,
              { {"right at the exit",    458.1, 0.0},
                {"50 cm after exit",     508.1, 0.0},
                {NULL, 0.0, 0.0} } },

            { "S bend", L_sbend, 4, 22.5, 0.0, 50.0,
              { {"right at turn in",     177.5, 0.0},
                {"right at the exit",    390.1, 0.0},
                {"50 cm after exit",     440.1, 0.0},
                {NULL, 0.0, 0.0} } },

            { "decreasing radius", L_decreasing, 4, 22.5, 0.0, 50.0,
              { {"right at turn in",     177.5, 0.0},
                {"right at the exit",    367.7, 0.0},
                {"50 cm after exit",     417.7, 0.0},
                {NULL, 0.0, 0.0} } },

            { "increasing radius", L_increasing, 4, 22.5, 0.0, 50.0,
              { {"right at turn in",     177.5, 0.0},
                {"right at the exit",    356.0, 0.0},
                {NULL, 0.0, 0.0} } },

            { "double apex", L_doubleApex, 5, 22.5, 0.0, 50.0,
              { {"between the apexes",   308.3, 0.0},
                {"right at the exit",    429.1, 0.0},
                {"50 cm after exit",     479.1, 0.0},
                {NULL, 0.0, 0.0} } },

            { "mixed circuit", L_mixed, 11, 22.5, 12.0, 60.0,
              { {"first straight",       180.0, 0.0},
                {"out of the first",     377.0, 0.0},
                {"before the 180",       590.0, 0.0},
                {"out of the 180",       905.0, 0.0},
                {NULL, 0.0, 0.0} } },

            { "narrow track 35 cm", L_medium, 3, 17.5, 0.0, 45.0,
              { {"right at the exit",    332.5, 0.0},
                {"25 cm after exit",     357.5, 0.0},
                {NULL, 0.0, 0.0} } },

            { "wide track 60 cm", L_medium, 3, 30.0, 0.0, 45.0,
              { {"right at the exit",    340.0, 0.0},
                {"25 cm after exit",     365.0, 0.0},
                {NULL, 0.0, 0.0} } },
        };

        int  nl = (int)(sizeof(lay) / sizeof(lay[0]));
        int  li, xi;
        int  fails = 0, runs = 0, missed = 0, touched = 0;
        int  only_l = -1;
        double totIsec = 0.0, totCtl = 0.0;

        if (argc > 2) only_l = atoi(argv[2]);
        if (argc > 3 && strcmp(argv[3], "-vv") == 0) g_isecTrace = 2;

        /* ---------------------------------------------------------------
         * Part 1: every layout, from five places on the track.
         *
         * Where the car happens to be when it arrives at a corner is not a
         * detail, it is the test. A tune that only ever sees one starting
         * offset is being scored on one trajectory, and the search will
         * happily spend every centimetre of margin on the other four without
         * ever showing it.
         * -------------------------------------------------------------*/
        {
            /* As a fraction of the room the car actually has: half the track
             * less half the car. Quoting these in centimetres looks tidy and is
             * wrong - on a 35 cm track, 12 cm off centre already has a wheel over
             * the line before the car has moved. */
            static const double off[] = { -0.75, -0.40, 0.0, 0.40, 0.75 };
            int    oi;
            int    clean = 0, total = 0;
            double slowest = 0.0, sum = 0.0;

            printf("=== part 1: the layouts, from five starting offsets, no crossing ===\n");
            printf("%-20s", "layout");
            for (oi = 0; oi < 5; oi++) printf(" %+6.0f%%", off[oi] * 100.0);
            printf("   worst clr  verdict\n");

            for (li = 0; li < nl; li++)
            {
                double worst = 1e9;
                int    bad = 0;

                if ((only_l >= 0) && (li != only_l)) continue;
                printf("%-20s", lay[li].name);

                for (oi = 0; oi < 5; oi++)
                {
                    Result r;

                    xing_clear();
                    r = run(lay[li].segs, lay[li].n, &cam, lay[li].halfW,
                            off[oi] * (lay[li].halfW - 7.0),
                            lay[li].name, 0, lay[li].maxT);
                    total++;
                    if ((r.excursions == 0) && r.finished)
                    {
                        clean++;
                        printf(" %6.2f", r.lapTime);
                        sum += r.lapTime;
                        if (r.lapTime > slowest) slowest = r.lapTime;
                    }
                    else
                    {
                        bad++;
                        sum += lay[li].maxT;
                        printf("   ----");
                    }
                    if (r.minClear < worst) worst = r.minClear;
                }
                printf("    %+6.2f  %s\n", worst,
                       bad ? "*** LEFT THE TRACK ***" : "clean from every start");
                if (bad) fails++;
            }
            printf("\nPART1 clean=%d/%d total=%.2f slowest=%.2f\n\n",
                   clean, total, sum, slowest);
        }

        /* ---------------------------------------------------------------
         * Part 2: a four road crossing, everywhere one could really go.
         *
         * A crossing is a straight tile, so it never sits on a curve - but the
         * straight it sits on can be very short. The placements are quoted as
         * the gap between the end of the corner and the near edge of the
         * crossing: "right at the exit" means the black lines stop the instant
         * the corner does, and the car is still unwinding when it arrives.
         * -------------------------------------------------------------*/
        printf("=== part 2: a four road crossing in every part of every layout ===\n");
        printf("45 cm track unless said otherwise; crossing 45 cm wide, 40 cm bars.\n");
        printf("'cost' is what the crossing added over the same layout without one.\n\n");
        printf("%-20s %-22s %7s %7s %6s %6s %5s  %s\n",
               "layout", "crossing", "lap s", "cost s", "avg", "clr", "cross", "verdict");

        for (li = 0; li < nl; li++)
        {
            Result ctl;
            double base;

            if ((only_l >= 0) && (li != only_l)) continue;

            xing_clear();
            ctl  = run(lay[li].segs, lay[li].n, &cam, lay[li].halfW,
                       lay[li].startLat, lay[li].name, 0, lay[li].maxT);
            base = ctl.lapTime;

            printf("%-20s %-22s %7.2f %7s %6.1f %+6.2f %5s  %s\n",
                   lay[li].name, "(none)", ctl.lapTime, "-", ctl.avgSpeed,
                   ctl.minClear, "-",
                   (ctl.excursions == 0 && ctl.finished) ? "clean"
                                                         : "*** BASE FAILED ***");
            if (ctl.excursions != 0 || !ctl.finished) fails++;

            for (xi = 0; (xi < 6) && (lay[li].x[xi].where != NULL); xi++)
            {
                Result r;
                int    want = (lay[li].x[xi].second > 0.0) ? 2 : 1;
                int    bad;

                xing_clear();
                xing_add(lay[li].x[xi].at, 45.0, 40.0, 1);
                if (lay[li].x[xi].second > 0.0)
                {
                    xing_add(lay[li].x[xi].second, 45.0, 40.0, 1);
                }

                r = run(lay[li].segs, lay[li].n, &cam, lay[li].halfW,
                        lay[li].startLat, lay[li].name, 0, lay[li].maxT);
                runs++;

                bad = (r.excursions > 0) || !r.finished || (r.isecCross != want);
                if (bad) fails++;
                if (r.excursions > 0 || !r.finished) touched++;
                else if (r.isecCross != want) missed++;

                printf("%-20s %-22s %7.2f %+7.2f %6.1f %+6.2f %2d/%-2d  %s\n",
                       "", lay[li].x[xi].where, r.lapTime, r.lapTime - base,
                       r.avgSpeed, r.minClear, r.isecCross, want,
                       (r.excursions > 0) ? "*** TOUCHED A LINE ***"
                       : (!r.finished)    ? "*** STOPPED IN IT ***"
                       : (r.isecCross != want) ? "*** NOT RECOGNISED ***"
                                               : "clean, went straight over");

                totIsec += r.lapTime;
                totCtl  += base;
            }
            printf("\n");
        }

        xing_clear();
        printf("%d crossings over %d layouts: %d clean, %d not recognised, "
               "%d left the track or stopped.\n",
               runs, nl, runs - missed - touched, missed, touched);
        printf("total %.1fs against %.1fs without them (%+.1f%%).\n",
               totIsec, totCtl,
               (totCtl > 0.0) ? (100.0 * (totIsec - totCtl) / totCtl) : 0.0);
        /* A report, not a gate. Some of these placements are known to be
         * beyond the car - see the README - so this returns 0 and lets the
         * verdict column say what happened, the way -fault and -sweep do. */
        printf("%s\n", fails ? "not every run was clean - see the verdicts above"
                                : "every run clean");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "-isec") == 0)
    {
        int fails = 0;

        if (argc > 2 && strcmp(argv[2], "-v") == 0)  g_isecTrace = 1;
        if (argc > 2 && strcmp(argv[2], "-vv") == 0) g_isecTrace = 2;

        /* ---------------------------------------------------------------
         * A. The scan on its own.
         * A handful of black lines go in, one verdict comes out. No car and no
         * circuit - this is the gap test and its guards, and nothing else.
         *
         * The cases are written where the lines really are, in centimetres ahead
         * of and to the left of the camera, and then projected into the frame and
         * rounded to whole pixels the way the Pixy2 would report them.
         * -------------------------------------------------------------*/
        printf("=== A. the gap scan ===\n");
        printf("%-38s %-6s %-6s %-8s %-8s %s\n",
               "case (45 cm track, camera 18 cm up)",
               "seen", "side", "gap cm", "steer", "verdict");
        {
            /* Each case is up to four ground lines: near edge, far edge, and
             * whatever else is in the frame. f0/f1 are how far ahead each end is,
             * l0/l1 how far to the left. */
            struct Line { double f0, l0, f1, l1; };
            struct Case
            {
                const char *name;
                struct Line ln[4];
                int         nLines;
                int         wantSeen;
                int         wantSide; /* -1 left, +1 right, 0 either */
            } cs[] = {
                { "left edge broken by a crossing",
                  { { 25.0, 22.5, 100.0, 22.5 }, { 145.0, 22.5, 200.0, 22.5 } },
                  2, 1, -1 },
                { "right edge broken by a crossing",
                  { { 25.0,-22.5, 100.0,-22.5 }, { 145.0,-22.5, 200.0,-22.5 } },
                  2, 1, +1 },
                { "both edges broken, with the bars",
                  { { 25.0, 22.5, 100.0, 22.5 }, { 145.0, 22.5, 200.0, 22.5 },
                    { 25.0,-22.5, 100.0,-22.5 }, { 145.0,-22.5, 200.0,-22.5 } },
                  4, 1, 0 },
                { "car 15 deg off, crossing ahead",
                  { { 25.0, 15.0, 100.0, 35.0 }, { 145.0, 47.0, 190.0, 59.0 } },
                  2, 1, -1 },
                { "edge in three pieces, one crossing",
                  { { 25.0, 22.5,  60.0, 22.5 }, {  60.0, 22.5, 100.0, 22.5 },
                    { 145.0, 22.5, 200.0, 22.5 } },
                  3, 1, -1 },

                /* things that must NOT trigger */
                { "unbroken edge, no crossing at all",
                  { { 25.0, 22.5, 200.0, 22.5 }, { 25.0,-22.5, 200.0,-22.5 } },
                  2, 0, 0 },
                { "edge just runs out of look-ahead",
                  { { 25.0, 22.5, 100.0, 22.5 } },
                  1, 0, 0 },
                { "10 cm break, camera dropped a bit",
                  { { 25.0, 22.5, 100.0, 22.5 }, { 110.0, 22.5, 200.0, 22.5 } },
                  2, 0, 0 },
                { "140 cm hole, far too big",
                  { { 25.0, 22.5,  40.0, 22.5 }, { 180.0, 22.5, 200.0, 22.5 } },
                  2, 0, 0 },
                { "far piece not parallel",
                  { { 25.0, 22.5, 100.0, 22.5 }, { 145.0, 22.5, 190.0, 62.0 } },
                  2, 0, 0 },
                { "far piece 40 cm out of line",
                  { { 25.0, 22.5, 100.0, 22.5 }, { 145.0,-17.5, 200.0,-17.5 } },
                  2, 0, 0 },
                { "edge starts 100 cm out, none near",
                  { { 100.0, 22.5, 130.0, 22.5 }, { 170.0, 22.5, 200.0, 22.5 } },
                  2, 0, 0 },
                { "only the bars, no edges at all",
                  { { 100.0, 22.5, 100.0, 62.5 }, { 100.0,-22.5, 100.0,-62.5 } },
                  2, 0, 0 },

                /* on the doorstep: both lines stop together, far side not in
                 * frame, but the mouth of the crossing is lying across the track */
                { "both stop at 50 cm, bar across",
                  { { 25.0, 22.5,  50.0, 22.5 }, { 25.0,-22.5,  50.0,-22.5 },
                    { 50.0, 22.5,  50.0, 62.5 } },
                  3, ISEC_MOUTH_ENABLE, 0 },
                { "both stop at 50 cm, nothing across",
                  { { 25.0, 22.5,  50.0, 22.5 }, { 25.0,-22.5,  50.0,-22.5 } },
                  2, 0, 0 },
                { "one line stops, the other carries on",
                  { { 25.0, 22.5,  50.0, 22.5 }, { 25.0,-22.5, 190.0,-22.5 },
                    { 50.0, 22.5,  50.0, 62.5 } },
                  3, 0, 0 },
                { "lines stop 30 cm apart, so a corner",
                  { { 15.0, 22.5,  25.0, 22.5 }, { 20.0,-22.5,  55.0,-22.5 },
                    { 25.0, 22.5,  25.0, 62.5 } },
                  3, 0, 0 },
                { "one line stops, other not in frame",
                  { { 25.0, 22.5,  50.0, 22.5 }, { 50.0, 22.5, 50.0, 62.5 } },
                  2, ISEC_MOUTH_ENABLE && ISEC_MOUTH_ONE_SIDED,
                  (ISEC_MOUTH_ENABLE && ISEC_MOUTH_ONE_SIDED) ? -1 : 0 },
                { "the one line sweeps away, so a bend",
                  { { 25.0, 22.5,  50.0, 35.0 }, { 50.0, 35.0, 50.0, 75.0 } },
                  2, 0, 0 },
            };
            int i;

            for (i = 0; i < (int)(sizeof(cs) / sizeof(cs[0])); i++)
            {
                TrkSegment   sg[4];
                Intersection ix;
                int          side, ok, q;

                for (q = 0; q < cs[i].nLines; q++)
                {
                    gproject(cs[i].ln[q].f0, cs[i].ln[q].l0, &sg[q].x0, &sg[q].y0);
                    gproject(cs[i].ln[q].f1, cs[i].ln[q].l1, &sg[q].x1, &sg[q].y1);
                }

                Track_Init();
                Intersection_Init(&ix);
                Intersection_Update(sg, (uint8_t)cs[i].nLines, &ix);

                side = ix.seen ? (ix.sawLeft ? -1 : +1) : 0;
                ok   = ((int)ix.seen == cs[i].wantSeen) &&
                       ((cs[i].wantSide == 0) || (side == cs[i].wantSide));
                if (!ok) fails++;

                printf("%-38s %-6s %-6s %-8.0f %+-8.1f %s\n",
                       cs[i].name,
                       ix.seen ? "yes" : "no",
                       ix.seen ? (ix.sawLeft ? (ix.sawRight ? "both" : "left")
                                             : "right")
                               : "-",
                       ix.seen ? ix.gapCm : 0.0,
                       ix.steer,
                       ok ? "ok" : "*** WRONG ***");
            }
        }

        /* The steering is a heading controller, so check it points the right way:
         * a track running off to the left ahead has to produce left lock. */
        {
            TrkSegment   sg[2];
            Intersection ix;
            int          ok;

            gproject(25.0, 15.0, &sg[0].x0, &sg[0].y0);   /* track heads LEFT ahead */
            gproject(120.0, 45.0, &sg[0].x1, &sg[0].y1);
            gproject(25.0, -30.0, &sg[1].x0, &sg[1].y0);
            gproject(120.0,  0.0, &sg[1].x1, &sg[1].y1);
            Track_Init();
            Intersection_Init(&ix);
            Intersection_Update(sg, 2, &ix);
            ok = (ix.steer < -1.0f);
            if (!ok) fails++;
            printf("%-38s %-6s %-6s %-8.2f %+-8.1f %s\n",
                   "track runs left ahead -> left lock", "-", "-",
                   ix.slope, ix.steer, ok ? "ok" : "*** WRONG ***");

            gproject(25.0, -15.0, &sg[0].x0, &sg[0].y0);  /* track heads RIGHT ahead */
            gproject(120.0, -45.0, &sg[0].x1, &sg[0].y1);
            gproject(25.0,  30.0, &sg[1].x0, &sg[1].y0);
            gproject(120.0,   0.0, &sg[1].x1, &sg[1].y1);
            Track_Init();
            Intersection_Init(&ix);
            Intersection_Update(sg, 2, &ix);
            ok = (ix.steer > 1.0f);
            if (!ok) fails++;
            printf("%-38s %-6s %-6s %-8.2f %+-8.1f %s\n",
                   "track runs right ahead -> right lock", "-", "-",
                   ix.slope, ix.steer, ok ? "ok" : "*** WRONG ***");
        }

        /* ---------------------------------------------------------------
         * B. False alarms.
         * Ordinary circuits with no crossing anywhere on them. One committed
         * crossing here is one place the car would drive straight on at a corner,
         * so the only acceptable number is zero.
         * -------------------------------------------------------------*/
        printf("\n=== B. false alarms on circuits with no crossing ===\n");
        printf("%-32s %-8s %-9s %-8s %s\n",
               "circuit", "frames", "detected", "latched", "verdict");
        {
            static const TrackSeg mixed[] = {
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
            static const TrackSeg tight[] = {
                {0.0, 200.0},
                {1.0 / 55.0, 55.0 * M_PI},
                {0.0, 200.0},
                {-1.0 / 80.0, 80.0 * M_PI},
                {0.0, 100.0},
            };
            static const TrackSeg oval[] = {
                {0.0, 300.0},
                {1.0 / 90.0, 90.0 * M_PI},
                {0.0, 300.0},
                {-1.0 / 90.0, 90.0 * M_PI},
                {0.0, 100.0},
            };
            static const TrackSeg narrow[] = {
                {0.0, 250.0},
                {1.0 / 60.0, 60.0 * M_PI},
                {0.0, 250.0},
                {-1.0 / 85.0, 85.0 * M_PI},
                {0.0, 100.0},
            };
            static const TrackSeg chicane[] = {
                {0.0, 250.0},
                {1.0 / 200.0, 60.0},
                {-1.0 / 200.0, 120.0},
                {1.0 / 200.0, 60.0},
                {0.0, 250.0},
                {1.0 / 70.0, 70.0 * M_PI},
                {0.0, 150.0},
                {-1.0 / 95.0, 95.0 * M_PI},
                {0.0, 100.0},
            };
            struct { const char *name; const TrackSeg *t; int n; double hw; double lat; } k[] = {
                { "mixed circuit, off centre", mixed,   11, 22.5, 12.0 },
                { "tight 180s",                tight,    5, 22.5,  0.0 },
                { "oval, R=90cm",              oval,     5, 22.5,  0.0 },
                { "narrow track (35 cm)",      narrow,   5, 17.5,  0.0 },
                { "chicane + sweepers",        chicane,  9, 22.5,  0.0 },
            };
            int i;

            xing_clear();
            for (i = 0; i < 5; i++)
            {
                Result r = run(k[i].t, k[i].n, &cam, k[i].hw, k[i].lat,
                               k[i].name, 0, 60.0);
                int bad = (r.isecCross > 0);

                if (bad) fails++;
                printf("%-32s %-8d %-9d %-8d %s\n", k[i].name, r.steps / 4,
                       r.isecSeen, r.isecCross,
                       bad ? "*** FALSE ALARM ***" : "clean");
            }
        }

        /* ---------------------------------------------------------------
         * C. A crossing that is really there.
         *
         * The main edges stop for the width of the crossing track and pick up
         * again on the far side, with four bars run out of the corners it leaves.
         * The car has to spot the hole, line itself up, and come out the far side
         * still on its own piece of track.
         *
         * Each case is run twice: once with the crossing and once with the same
         * stretch of track left whole. Without that control there is no telling
         * whether an offset came from the crossing or from the racing line, which
         * uses the width of the track on purpose.
         * -------------------------------------------------------------*/
        printf("\n=== C. driving through a crossing ===\n");
        printf("%-34s %-9s %-8s %-9s %-8s %-8s %s\n",
               "case", "detected", "latched", "at (cm)",
               "pk|lat|", "no cross", "verdict");
        {
            static const TrackSeg straight[] = {
                {0.0, 200.0},
                {0.0, 250.0},
                {0.0, 250.0},
            };
            static const TrackSeg afterBend[] = {
                {0.0, 200.0},
                {1.0 / 90.0, 90.0 * (M_PI / 3)},
                {0.0, 300.0},
                {0.0, 200.0},
            };
            struct { const char *name; const TrackSeg *t; int n; double at;
                     double lat; double bar; int farEdge; int wantCross; } k[] = {
                { "straight, crossing at 300 cm", straight,  3, 300.0,   0.0, 40.0, 1, 1 },
                { "same, car starts 10 cm left",  straight,  3, 300.0,  10.0, 40.0, 1, 1 },
                { "same, car starts 10 cm right", straight,  3, 300.0, -10.0, 40.0, 1, 1 },
                { "short bars, 20 cm stubs",      straight,  3, 300.0,   0.0, 20.0, 1, 1 },
                /* Since TRK_EXTRAP_SPAN_K stopped the corridor breaking, this one
                 * is usually driven straight over without the latch ever firing -
                 * so what is asserted is the outcome, not the mechanism. -1 means
                 * either way is fine as long as the crossing does not disturb the
                 * car, which the offset-against-control check below enforces. */
                { "crossing just after a bend",   afterBend, 4, 420.0,   0.0, 40.0, 1, -1 },
                /* The camera never reports the far side, so the only thing left to
                 * go on is both lines stopping together in front of the car. */
                { "far side never reported at all", straight, 3, 300.0,  0.0, 40.0, 0,
                  ISEC_MOUTH_ENABLE },
                /* ...and off centre the other line is outside a 60 degree view as
                 * well, so there is only one left. Recognising that needs
                 * ISEC_MOUTH_ONE_SIDED, which ships off - see race_config.h. */
                { "far side gone, car 10 cm left", straight, 3, 300.0, 10.0, 40.0, 0,
                  ISEC_MOUTH_ENABLE && ISEC_MOUTH_ONE_SIDED },
            };
            int i;

            for (i = 0; i < 7; i++)
            {
                Result r, ctl;
                int    bad;

                g_winLo = (int)((k[i].at - 100.0) / g_step);
                g_winHi = (int)((k[i].at + 100.0) / g_step);

                xing_clear();
                ctl = run(k[i].t, k[i].n, &cam, 22.5, k[i].lat, k[i].name, 0, 40.0);

                xing_clear();
                xing_add(k[i].at + 22.5, 45.0, k[i].bar, k[i].farEdge);
                r = run(k[i].t, k[i].n, &cam, 22.5, k[i].lat, k[i].name, 0, 40.0);

                g_winLo = -1;
                g_winHi = -1;

                /* A case that is meant to be crossed has to finish. One that
                 * is not only has to stay inside the lines - stopping in the
                 * middle of an unrecognised crossing is the failsafe doing
                 * its job, not a fault. */
                bad = (r.excursions > 0) ||
                      ((k[i].wantCross > 0) && !r.finished) ||
                      ((k[i].wantCross >= 0) &&
                       ((r.isecCross >= 1) != (k[i].wantCross != 0)));

                /* Whichever way it got through, the crossing must not have moved
                 * the car off the line it would have taken without one. */
                if ((k[i].wantCross != 0) && r.finished &&
                    (fabs(r.peakLatWin - ctl.peakLatWin) > 3.0))
                {
                    bad = 1;
                }
                if (bad) fails++;

                printf("%-34s %-9d %-8d %-9.0f %-8.1f %-8.1f %s\n", k[i].name,
                       r.isecSeen, r.isecCross, r.isecFirstS,
                       r.peakLatWin, ctl.peakLatWin,
                       bad ? "*** FAILED ***"
                           : (k[i].wantCross
                                  ? ((r.isecCross >= 1)
                                         ? "straight through, inside the lines"
                                         : "corridor held, no latch needed")
                                  : (r.finished
                                         ? "not recognised, driven as track"
                                         : "not recognised, stopped safely")));
            }
            xing_clear();
        }

        printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all intersection checks passed");
        return fails ? 1 : 0;
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
