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
#include "intersection.h"

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
/* Intersections                                                       */
/*                                                                     */
/* A crossing is a second piece of track laid over ours. Work in local  */
/* coordinates at the crossing centre:                                  */
/*                                                                      */
/*    u  along our track (forward)                                      */
/*    v  across it (+v is to the left)                                  */
/*                                                                      */
/* Our track occupies |v| <= halfW and its two black lines sit at       */
/* v = +/-halfW. The crossing track runs along d = (cos a, sin a) and    */
/* occupies |(u,v).n| <= halfWx, where n = (-sin a, cos a); its own two  */
/* lines sit on (u,v).n = +/-halfWx.                                     */
/*                                                                      */
/* Both tracks are painted on the same floor, so in the overlap the      */
/* lines are interrupted - ours where the crossing track's surface       */
/* covers them, theirs where ours does. That is the whole geometry, and  */
/* it is what the car has to recognise:                                  */
/*                                                                      */
/*    our two lines stop, leave a gap, and resume in line with           */
/*    themselves, and a pair of roughly sideways lines lies across       */
/*    the gap                                                            */
/* ------------------------------------------------------------------ */
#define XSEC_MAX 8

typedef struct
{
    double s;      /* arc position of the crossing centre, cm            */
    double ang;    /* crossing angle to our track, rad (M_PI/2 = square) */
    double halfW;  /* half width of the crossing track, cm               */
} Crossing;

static Crossing g_xs[XSEC_MAX];
static int      g_nxs = 0;

static void xsec_clear(void)
{
    g_nxs = 0;
}

static void xsec_add(double s, double angDeg, double halfW)
{
    if (g_nxs >= XSEC_MAX) return;
    g_xs[g_nxs].s     = s;
    g_xs[g_nxs].ang   = angDeg * M_PI / 180.0;
    g_xs[g_nxs].halfW = halfW;
    g_nxs++;
}

/* Local (u,v) of a world point about crossing k. */
static void xsec_local(int k, double px, double py, double *u, double *v)
{
    int    i  = (int)(g_xs[k].s / g_step);
    double th, dx, dy;

    if (i < 0) i = 0;
    if (i >= g_cn) i = g_cn - 1;

    th = g_cth[i];
    dx = px - g_cx[i];
    dy = py - g_cy[i];

    *u = (cos(th) * dx) + (sin(th) * dy);
    *v = (-sin(th) * dx) + (cos(th) * dy);
}

/* World point from local (u,v) about crossing k. */
static void xsec_world(int k, double u, double v, double *px, double *py)
{
    int    i = (int)(g_xs[k].s / g_step);
    double th;

    if (i < 0) i = 0;
    if (i >= g_cn) i = g_cn - 1;

    th  = g_cth[i];
    *px = g_cx[i] + (cos(th) * u) - (sin(th) * v);
    *py = g_cy[i] + (sin(th) * u) + (cos(th) * v);
}

/* Which crossing paints over one of our own black lines here, or -1. */
static int xsec_which_hides(double px, double py)
{
    int k;

    for (k = 0; k < g_nxs; k++)
    {
        double u, v, perp;

        xsec_local(k, px, py, &u, &v);
        if (fabs(u) > 200.0) continue; /* nowhere near this crossing */

        perp = (-u * sin(g_xs[k].ang)) + (v * cos(g_xs[k].ang));
        if (fabs(perp) <= g_xs[k].halfW)
        {
            return k;
        }
    }
    return -1;
}

/*
 * Does the camera follow the corner?
 *
 * Where our black line runs into the crossing's black line the two meet and make
 * one continuous piece of paint. A line tracker following ours does not stop at
 * the join - it turns and carries on outward along theirs. That is what the real
 * Pixy2 does, and it is why a crossing arrives as two long vectors splaying
 * apart rather than as a set of loose horizontal bars.
 *
 * Set to 0 to render the two lines as separate pieces instead. Which one a real
 * camera gives depends on how it segments the frame, so the detector is tested
 * against both.
 */
static int g_xsecMerge = 1;

/* Draw only one of the crossing track's two stubs on each of its lines: -1
 * keeps the v<0 side, +1 the v>0 side, 0 both. With g_xsecMerge off this
 * is the frame a real Pixy2 gave at a junction: our line stopping dead at
 * the corner, the crossing edge leaving that corner on ONE side, and a
 * plain line on the other - a single elbow, nothing opposite it. */
static int g_xsecStub = 0;

/*
 * Where our line at v = side*halfW runs into one of the crossing's two lines,
 * and which of the two it meets first coming from the car.
 *
 * The crossing's lines are the two solutions of (u,v).n = +/-halfWx; ours cuts
 * each of them at one point, and the corner that matters is the nearer one.
 */
static int xsec_corner(int k, double side, double *tCorner, double *cSel, double *dirOut)
{
    double a  = g_xs[k].ang;
    double sA = sin(a), cA = cos(a);
    double best = 0.0, bestT = 0.0, bestU = 1e18;
    int    found = 0, i;

    if (fabs(sA) < 0.15) return 0; /* nearly parallel to us - not a crossing */

    for (i = 0; i < 2; i++)
    {
        double c = (i == 0) ? g_xs[k].halfW : -g_xs[k].halfW;
        /* v(t) = t*sA + c*cA = side*g_halfW */
        double t = ((side * g_halfW) - (c * cA)) / sA;
        double u = (t * cA) - (c * sA);

        if (u < bestU) { bestU = u; bestT = t; best = c; found = 1; }
    }

    if (!found) return 0;

    *tCorner = bestT;
    *cSel    = best;
    /* Outward means |v| growing on our own side of the track. */
    *dirOut  = ((side * sA) >= 0.0) ? 1.0 : -1.0;
    return 1;
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

/*
 * Renders one track edge into up to 3 segments, the way the Pixy2 breaks a curved
 * line into a small chain of vectors. Points are quantised to integers first.
 */
static int g_fitChunks = 0;
#define FIT_TOL 1.0 /* cells a chain may stray from its chord and still be one vector */

/* Chords fitted to the chain: one vector while every point sits within
 * FIT_TOL of it, otherwise split at the point that strays farthest. */
static int emit_fit(const double *ux, const double *vy, int a, int b,
                    TrkSegment *out, int maxOut)
{
    double x0 = ux[a], y0 = vy[a], x1 = ux[b], y1 = vy[b];
    double dx = x1 - x0, dy = y1 - y0, len = sqrt((dx * dx) + (dy * dy));
    double worst = 0.0;
    int    wi = -1, i, n1;

    if (maxOut < 1) return 0;
    if ((b - a) >= 2 && len > 0.5 && maxOut >= 2)
    {
        for (i = a + 1; i < b; i++)
        {
            double d = fabs(((ux[i] - x0) * dy) - ((vy[i] - y0) * dx)) / len;
            if (d > worst) { worst = d; wi = i; }
        }
    }
    if (wi < 0 || worst <= FIT_TOL)
    {
        out[0].x0 = (float)x0; out[0].y0 = (float)y0;
        out[0].x1 = (float)x1; out[0].y1 = (float)y1;
        return 1;
    }
    n1 = emit_fit(ux, vy, a, wi, out, maxOut - 1);
    return n1 + emit_fit(ux, vy, wi, b, out + n1, maxOut - n1);
}

/* Splits one chain of image points into up to 3 straight vectors. */
static int emit_chunks(const double *ux, const double *vy, int np,
                       TrkSegment *out, int maxOut)
{
    int chunks, c, nseg = 0;

    if (np < 2 || maxOut < 1) return 0;
    if (g_fitChunks) return emit_fit(ux, vy, 0, np - 1, out, maxOut);

    chunks = (np >= 12) ? 3 : ((np >= 6) ? 2 : 1);
    if (chunks > g_maxChunks) chunks = g_maxChunks;
    /* Fewer, longer chords rather than dropping the far end of the run:
     * a camera does not discard the part of a line nearest a corner. */
    if (chunks > maxOut) chunks = maxOut;

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

#define MAX_RUNS 6

/*
 * Carries the point chain round the corner and out along the crossing's own
 * line, which is what makes the vector splay outward instead of stopping.
 * Returns the new point count.
 */
static int append_flare(const Cam *cam, double px, double py, double pth,
                        int k, double side, double *ux, double *vy, int np)
{
    double tC, c, dir, t;
    double a, dU, dV, nU, nV;

    if (!xsec_corner(k, side, &tC, &c, &dir)) return np;

    a  = g_xs[k].ang;
    dU = cos(a); dV = sin(a);
    nU = -sin(a); nV = cos(a);

    for (t = tC; np < 128; t += dir * 1.0)
    {
        double u = (t * dU) + (c * nU);
        double v = (t * dV) + (c * nV);
        double wx, wy, dx, dy, fwd, lat, iu, iv;

        if (fabs(v) > 160.0) break;
        if (fabs(v) < g_halfW) continue; /* still over our own track */

        xsec_world(k, u, v, &wx, &wy);
        dx = wx - px; dy = wy - py;
        fwd = (cos(pth) * dx) + (sin(pth) * dy);
        lat = (-sin(pth) * dx) + (cos(pth) * dy);

        if (fwd < 1.0 || fwd > 250.0) break;
        if (!project(cam, fwd, lat, &iu, &iv)) break;

        iu = floor(iu + 0.5);
        iv = floor(iv + 0.5);
        if (np > 0 && iu == ux[np - 1] && iv == vy[np - 1]) continue;
        ux[np] = iu; vy[np] = iv; np++;
    }
    return np;
}

static int render_edge(const Cam *cam, double px, double py, double pth, int hint,
                       double side, TrkSegment *out, int maxOut)
{
    double ux[128], vy[128];
    int    rs[MAX_RUNS], rl[MAX_RUNS];
    int    nr = 0, np = 0, cur = 0;
    int    nseg = 0, r, per;
    int    idx;
    int    flared = 0;

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

        /* A crossing track is painted over our line here. Our line stops - but
         * the crossing's own line starts at exactly that corner, so unless the
         * camera breaks them apart the vector turns and carries on outward.
         * That flare is the thing the car recognises. */
        if (g_nxs > 0)
        {
            int kh = xsec_which_hides(ex, ey);

            if (kh >= 0)
            {
                if (g_xsecMerge && !flared)
                {
                    int before = np;

                    np = append_flare(cam, px, py, pth, kh, side, ux, vy, np);
                    if (np > before)
                    {
                        flared = 1;
                    }
                }
                if (((np - cur) >= 2) && (nr < MAX_RUNS))
                {
                    rs[nr] = cur; rl[nr] = np - cur; nr++;
                }
                cur = np;
                continue;
            }
        }

        if (!project(cam, fwd, lat, &u, &v)) continue;

        u = floor(u + 0.5);
        v = floor(v + 0.5);
        if (np > cur && u == ux[np - 1] && v == vy[np - 1]) continue;
        ux[np] = u; vy[np] = v; np++;
    }

    if (((np - cur) >= 2) && (nr < MAX_RUNS))
    {
        rs[nr] = cur; rl[nr] = np - cur; nr++;
    }
    if (nr == 0) return 0;

    /* Share the vector budget across the runs, so the piece of line beyond a
     * crossing is still reported - it is the evidence that the track
     * continues - but the near run is served first and in full. Splitting
     * the budget evenly gave each run two of its three chords and silently
     * dropped the third, which for the near run is the part that reaches
     * the corner: no corner, no elbow, whatever the camera would have
     * drawn. */
    {
        int want[MAX_RUNS], give[MAX_RUNS], left = maxOut;

        for (r = 0; r < nr; r++)
        {
            want[r] = (rl[r] >= 12) ? 3 : ((rl[r] >= 6) ? 2 : 1);
            if (want[r] > g_maxChunks) want[r] = g_maxChunks;
            give[r] = 0;
        }
        for (r = 0; r < nr && left > 0; r++) { give[r] = 1; left--; }
        for (r = 0; r < nr && left > 0; r++)
        {
            while (give[r] < want[r] && left > 0) { give[r]++; left--; }
        }
        for (r = 0; r < nr && nseg < maxOut; r++)
        {
            if (give[r] > 0)
                nseg += emit_chunks(&ux[rs[r]], &vy[rs[r]], rl[r], out + nseg, give[r]);
        }
    }
    (void)per;
    return nseg;
}

/*
 * The crossing track's own two black lines. They run across our path, and our
 * track paints over their middle, so each one reaches the camera as two stubs -
 * one either side of our corridor.
 */
static int render_crossings(const Cam *cam, double px, double py, double pth,
                            TrkSegment *out, int maxOut)
{
    int k, nseg = 0;

    for (k = 0; k < g_nxs && nseg < maxOut; k++)
    {
        double a  = g_xs[k].ang;
        double dU = cos(a), dV = sin(a);  /* along the crossing track */
        double nU = -sin(a), nV = cos(a); /* across it                */
        int    sgn, sideSel;

        for (sgn = -1; sgn <= 1 && nseg < maxOut; sgn += 2)
        {
            for (sideSel = 0; sideSel < 2 && nseg < maxOut; sideSel++)
            {
                double uxa[128], vya[128];
                int    np = 0, cap;
                double t;

                /* When the camera follows the corner, this stub has already
                 * been drawn as the outward half of our own line's vector.
                 * Drawing it again would put two vectors on one piece of paint. */
                if (g_xsecMerge)
                {
                    double tC, cS, dr;
                    double sd = (sideSel == 1) ? 1.0 : -1.0;

                    if (xsec_corner(k, sd, &tC, &cS, &dr) &&
                        (fabs(cS - ((double)sgn * g_xs[k].halfW)) < 1e-6))
                    {
                        continue;
                    }
                }

                for (t = -160.0; t <= 160.0; t += 1.0)
                {
                    double u = (t * dU) + ((double)sgn * g_xs[k].halfW * nU);
                    double v = (t * dV) + ((double)sgn * g_xs[k].halfW * nV);
                    double wx, wy, dx, dy, fwd, lat, iu, iv;

                    if (fabs(v) <= g_halfW) continue;          /* our track covers it */
                    if ((sideSel == 0) && (v > 0.0)) continue; /* one stub at a time  */
                if ((g_xsecStub < 0) && (v > 0.0)) continue; /* that stub not drawn */
                if ((g_xsecStub > 0) && (v < 0.0)) continue;
                    if ((sideSel == 1) && (v < 0.0)) continue;

                    xsec_world(k, u, v, &wx, &wy);
                    dx = wx - px; dy = wy - py;
                    fwd = (cos(pth) * dx) + (sin(pth) * dy);
                    lat = (-sin(pth) * dx) + (cos(pth) * dy);

                    if (fwd < 1.0 || fwd > 250.0) continue;
                    if (!project(cam, fwd, lat, &iu, &iv)) continue;

                    iu = floor(iu + 0.5);
                    iv = floor(iv + 0.5);
                    if (np > 0 && iu == uxa[np - 1] && iv == vya[np - 1]) continue;
                    if (np < 128) { uxa[np] = iu; vya[np] = iv; np++; }
                }

                cap = maxOut - nseg;
                if (cap > 2) cap = 2;
                nseg += emit_chunks(uxa, vya, np, out + nseg, cap);
            }
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
    int    oeRunMax;        /* longest unbroken one-sided run, frames  */
    double oeMinClearMiss;  /* closest to the UNSEEN line while blind  */
    double oeDriftSum;      /* |lat| accumulated over one-sided frames */
    int    oeDriftN;
    int    pbFrames;        /* frames with the recovery probe leaning  */
    double pbMinClear;      /* worst clearance while it was leaning    */
    int    pbStarted;
    int    pbFound;
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
/* A yaw step injected once, when the car passes g_kickAt cm along the track:
 * the car arriving at a junction still correcting for something - a bump, a
 * skid, the tail of a bend. Degrees, + = to the left. */
static double g_kickAt  = -1.0;
static double g_kickDeg = 0.0;
static int    g_kicked  = 0;
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

/*
 * One-sided vision diagnostics.
 *
 * When only one black line is measured the corridor on the other side is a guess
 * from the width model, so the number that matters is how close the car got to the
 * line it could NOT see - the one it has no feedback on.
 */
static void oe_score(Result *r, const DriveState *st, double lat, double halfW,
                     double halfCar, int *run)
{
    int sawL = 0, sawR = 0, i;
    double clr;

    if (!st->track.haveTrack || st->track.bothEdges)
    {
        *run = 0;
        return;
    }

    for (i = 0; i < (int)st->track.nValid; i++)
    {
        if (st->track.sawL[i]) sawL = 1;
        if (st->track.sawR[i]) sawR = 1;
    }
    if (sawL == sawR) { *run = 0; return; }

    (*run)++;
    if (*run > r->oeRunMax) r->oeRunMax = *run;

    /* +lat is toward the LEFT line. */
    clr = sawL ? (halfW + lat - halfCar)   /* the right line is the unseen one */
               : (halfW - lat - halfCar);
    if (clr < r->oeMinClearMiss) r->oeMinClearMiss = clr;

    r->oeDriftSum += fabs(lat);
    r->oeDriftN++;
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
    int    oeRun = 0;


    memset(&r, 0, sizeof(r));
    r.name = name;
    r.minClear = 1e9;
    r.oeMinClearMiss = 1e9;
    r.pbMinClear = 1e9;
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
    g_kicked = 0;

    while (t < maxTime)
    {
        bool    fresh;
        uint8_t n = 0;
        int     idx;
        double  lat, clear;

        idx  = nearest_idx(car.x, car.y, hint);
        hint = idx;

        if ((g_kickAt >= 0.0) && !g_kicked && (idx >= (int)(g_kickAt / g_step)))
        {
            car.th += g_kickDeg * M_PI / 180.0;
            g_kicked = 1;
        }

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
            k = render_edge(cam, sx, sy, sth, sidx, -1.0, allSegs + n, 4);
            n = (uint8_t)(n + k);
            if (g_nxs > 0)
            {
                k = render_crossings(cam, sx, sy, sth, allSegs + n,
                                     PIXY_MAX_VECTORS - n);
                n = (uint8_t)(n + k);
            }
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
            oe_score(&r, st, lateral_offset(car.x, car.y, idx), halfW,
                     car.halfCar, &oeRun);
            if (st->rcv.active)
            {
                double c = halfW - fabs(lateral_offset(car.x, car.y, idx)) - car.halfCar;
                r.pbFrames++;
                if (c < r.pbMinClear) r.pbMinClear = c;
            }
            r.pbStarted = (int)st->rcv.probes;
            r.pbFound   = (int)st->rcv.found;
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
            printf("-- t=%.3f s=%.0fcm pose lat=%+.2f  segs=%d\n", t, idx * g_step, lat, (int)n);
            for (q = 0; q < (int)n; q++)
                printf("     seg%d (%.0f,%.0f)->(%.0f,%.0f)\n", q,
                       allSegs[q].x0, allSegs[q].y0, allSegs[q].x1, allSegs[q].y1);
            for (q = 0; q < TRK_ROWS; q++)
                printf("     row%d y=%2.0f v=%d L=%7.1f R=%7.1f c=%6.1f w=%6.1f sL=%d sR=%d\n",
                       q, st->track.y[q], st->track.valid[q], st->track.xl[q],
                       st->track.xr[q], st->track.center[q], st->track.width[q],
                       st->track.sawL[q], st->track.sawR[q]);
            printf("     hN=%+.3f hF=%+.3f nValid=%d tgt=%.1f la=%d steer=%.1f"
                   " segCount=%d conf=%.2f%s%s\n",
                   st->track.headNear, st->track.headFar, st->track.nValid,
                   st->line.targetX, st->line.laRow, cmd.steer,
                   st->track.segCount, st->line.conf,
                   st->line.straight ? " STRAIGHT" : "",
                   st->line.chorded ? " CHORDED" : "");
        }

        if (verbose && (r.steps % 25 == 0))
        {
            static const char *ph[] = { "-", "AHEAD", "CROSS", "clear" };
            const DriveState *st = Driver_State();
            printf("  t=%5.2f lat=%+6.2f clr=%+5.2f v=%5.1f steer=%+6.1f "
                   "hN=%+5.2f hF=%+5.2f rows=%d sev=%.2f "
                   "x=%-5s bars=%d y=%4.1f cov=%4.2f gap=%4.2f div=%4.2f%s%s%s%s\n",
                   t, lat, clear, car.v, cmd.steer,
                   st->track.headNear, st->track.headFar, st->track.nValid,
                   st->severity,
                   ph[(int)st->xsec.phase], st->xsec.bars, st->xsec.barY,
                   st->xsec.cover, st->xsec.gap, st->xsec.diverge,
                   st->xsec.bothSides ? " LR" : "",
                   st->xsec.spanning ? " SPAN" : "",
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
    printf("%-26s oneEdge=%d (run<=%d)  lost=%d  chicaneFrames=%d\n",
           "", r->oneEdgeFrames, r->oeRunMax, r->lostFrames, r->chicaneFrames);
    printf("%-26s blindSideClear=%+.2f cm  mean|lat| while blind=%.2f cm\n",
           "", (r->oeMinClearMiss > 1e8) ? 0.0 : r->oeMinClearMiss,
           (r->oeDriftN > 0) ? (r->oeDriftSum / r->oeDriftN) : 0.0);
    printf("%-26s probes=%d found=%d  leanFrames=%d  clearWhileLeaning=%+.2f cm\n\n",
           "", r->pbStarted, r->pbFound, r->pbFrames,
           (r->pbMinClear > 1e8) ? 0.0 : r->pbMinClear);
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

    /* Every mode, not just -chord, can be run at the vector count real hardware
     * produces: SIM_CHUNKS=1 merges each edge into one chord, 2 splits it once.
     * The modes that set g_maxChunks themselves still override this. */
    {
        const char *e = getenv("SIM_CHUNKS");

        if ((e != NULL) && (atoi(e) > 0))
        {
            g_maxChunks = atoi(e);
        }
    }

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

    if (argc > 1 && strcmp(argv[1], "-oneside") == 0)
    {
        /*
         * Driving on one black line and a guess.
         *
         * The Pixy2 sees ~60 degrees across, which is narrower than the track is
         * wide close up, so there are ordinary corners where the outside line
         * simply leaves the frame sideways and stays out of it. track.c infers
         * the missing edge from its width model and keeps going, which means the
         * car is steering off a number nothing is checking - and the further it
         * drifts, the more certain the one line it CAN see stays.
         *
         * The number that matters here is not lap time and not the clearance the
         * car ended up with. It is blindSideClear: how close it came to the line
         * it could not see, on the frames it could not see it. That is the one
         * with no feedback behind it.
         *
         * The corners get tighter down the table and the last two also narrow
         * the view, which is the same thing a camera mounted a little too high
         * or aimed a little too low does on a real car.
         */
        struct { const char *name; double f, h, hz, halfW, R; } k[] = {
            { "R90 bend, good mount",    68.0, 18.0, -4.0, 22.5, 90.0 },
            { "R75 bend",                68.0, 18.0, -4.0, 22.5, 75.0 },
            { "R60 bend",                68.0, 18.0, -4.0, 22.5, 60.0 },
            { "R60 bend, 55cm track",    68.0, 18.0, -4.0, 27.5, 60.0 },
            { "R75 bend, narrow view",   80.0, 18.0, -4.0, 22.5, 75.0 },
            { "R75 bend, aimed low",     68.0, 22.0, -8.0, 22.5, 75.0 },
        };
        int i;

        printf("=== one-sided vision: driving on one line and a guess ===\n");
        printf("RCV_ENABLE=%d.  blindSide is the clearance to the line the car could\n",
               (int)RCV_ENABLE);
        printf("NOT see, while it could not see it - the number with no feedback.\n\n");
        printf("%-26s %-6s %-8s %-9s %-10s %-8s %s\n", "case", "lap", "oneEdge",
               "run<=", "blindSide", "minClear", "probes");
        for (i = 0; i < 6; i++)
        {
            TrackSeg t[4];
            Cam    cm;
            Result r;

            t[0].curv = 0.0;         t[0].len = 200.0;
            t[1].curv = 1.0 / k[i].R; t[1].len = k[i].R * M_PI;
            t[2].curv = 0.0;         t[2].len = 150.0;
            t[3].curv = -1.0 / k[i].R; t[3].len = k[i].R * (M_PI / 2);

            cm.f = k[i].f; cm.h = k[i].h; cm.horiz = k[i].hz;
            g_rng = 12345u;
            g_dropRate = 0.0; g_blindFrom = -1.0; g_blindTo = -1.0;
            r = run(t, 4, &cm, k[i].halfW, 0.0, k[i].name, 0, 40.0);
            printf("%-26s %-6.2f %-8d %-9d %+-10.2f %+-9.2f %d/%d%s\n",
                   k[i].name, r.lapTime, r.oneEdgeFrames, r.oeRunMax,
                   (r.oeMinClearMiss > 1e8) ? 0.0 : r.oeMinClearMiss,
                   r.minClear, r.pbFound, r.pbStarted,
                   r.finished ? "" : "   *DNF*");
        }
        printf("\nRun this against a -DRCV_ENABLE=0 build; blindSide must not shrink.\n");
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

    /* Raw frames over a crossing, so the signature can be looked at directly
     * rather than guessed at.  -xdump [from] [to] [angle] */
    /*
     * Hand-built camera frames.
     *
     * The circuits in here render their own frames from a track and a camera, and
     * that is the right way to test almost everything - but it cannot produce the
     * shape a crossing makes at the top of a real Pixy2 frame. At every mounting
     * this simulator can actually drive with, our own two lines are already jammed
     * against the sides of the frame close up, so nothing has room to bend or splay
     * past them; aim far enough ahead to make room and the horizon guard cuts the
     * model to four rows and the car cannot drive at all.
     *
     * So the shapes are typed in instead, off a real PixyMon frame, and the
     * negatives are typed in beside them. The negatives are the point. Any detector
     * fires on the frame it was written from; what has to be shown is that the
     * things which look like a crossing and are not one - a right-angle corner in
     * our own track, a line broken by a gap in the paint, one elbow with nothing
     * opposite it - still come back quiet.
     */
    if (argc > 1 && strcmp(argv[1], "-frames") == 0)
    {
        struct
        {
            const char *name;
            int         want;  /* 1 = must be recognised, 0 = must not be,
                                * 2 = no test can see it; reported only   */
            int         nseg;
            double      s[8][4]; /* x0 y0 x1 y1, grid cells */
            int         camB;    /* branches the camera reports, 0 = silent */
            double      camX, camY; /* where it says the junction is        */
        } cases[] = {
            /* Measured off the PixyMon frame the user supplied: our two lines
             * each stop dead at a corner, and the crossing track's near edge
             * sets off sideways from that exact point, out to both sides. */
            { "crossing, elbows (the PixyMon frame)", 1, 4,
              { { 12.0, 51.6, 21.9,  2.4 },   /* left line, running away   */
                { 21.9,  2.4,  0.7,  7.2 },   /* its arm, out to the left  */
                { 64.1, 51.6, 57.1,  1.6 },   /* right line                */
                { 57.1,  1.6, 78.6,  3.8 } } },/* its arm, out to the right */

            /* Same junction, arms dead level - the slope in the frame above is
             * the lens, not the track, so neither reading may depend on it. */
            { "crossing, elbows, level arms", 1, 4,
              { { 12.0, 51.6, 21.9,  2.4 },
                { 21.9,  2.4,  0.7,  2.4 },
                { 64.1, 51.6, 57.1,  1.6 },
                { 57.1,  1.6, 78.6,  1.6 } } },

            /* The second real PixyMon frame: the camera gave exactly three
             * vectors. Our left line runs up to a sharp corner, the crossing
             * edge leaves that corner to the left, and the right-hand line is
             * plain and unbroken. A real junction with only one elbow in it -
             * and the car yawed some twenty degrees, so the corridor sweeps
             * 0.6 columns a row. Two readings of the same screenshot, the
             * second assuming it is cropped on the right. */
            { "crossing, ONE elbow (2nd PixyMon frame)", 1, 3,
              { {  0.5, 51.2, 37.2,  7.8 },   /* left line, running away   */
                { 37.4,  7.6,  7.1,  2.6 },   /* its arm, out to the left  */
                { 62.3, 51.4, 75.3, 21.4 } } },/* right line, no corner    */
            { "crossing, ONE elbow (2nd frame, re-read)", 1, 3,
              { {  0.0, 50.8, 34.1,  7.0 },
                { 34.1,  7.0,  6.1,  2.2 },
                { 57.0, 50.6, 69.0, 20.9 } } },

            /* The third real frame, the same junction close up: both our
             * lines stop at mid-frame and the crossing's far edge shows at
             * the top left. No elbow at all - the camera did not draw the
             * near edge - so no test here can fire on it. It is the frame
             * AFTER the one the car recognised, and carrying the recognition
             * across it is the XSEC_AHEAD state's job, not a detector's. */
            { "crossing close up, no elbow (3rd frame)", 2, 3,
              { { 18.0, 50.8, 14.0, 34.7 },
                {  9.7,  9.6, 28.3,  5.6 },
                { 71.7, 49.7, 69.7, 34.7 } } },

            /* Nothing but our own two lines, converging as perspective says
             * they must. */
            { "clean straight", 0, 2,
              { { 12.0, 51.6, 30.0,  2.0 },
                { 64.1, 51.6, 49.0,  2.0 } } },

            /* A right-angle corner in OUR track, turning left. Both lines end
             * in a corner and both arms sweep the same way, because they are
             * two sides of one road. This is the frame the whole test exists to
             * tell apart from the first one. */
            { "square corner, ours, turning left", 0, 4,
              { { 12.0, 51.6, 21.9,  2.4 },
                { 21.9,  2.4,  0.7,  7.2 },
                { 64.1, 51.6, 57.1,  1.6 },
                { 57.1,  1.6, 36.0,  6.0 } } },

            { "square corner, ours, turning right", 0, 4,
              { { 12.0, 51.6, 21.9,  2.4 },
                { 21.9,  2.4, 43.0,  7.0 },
                { 64.1, 51.6, 57.1,  1.6 },
                { 57.1,  1.6, 78.6,  3.8 } } },

            /* Each line broken in two by a gap in the paint. The pieces share an
             * endpoint exactly as an elbow does - what they do not do is head
             * off sideways from it. */
            { "line broken by a paint gap", 0, 4,
              { { 12.0, 51.6, 19.0, 12.0 },
                { 19.0, 12.0, 21.9,  2.4 },
                { 64.1, 51.6, 59.0, 12.0 },
                { 59.0, 12.0, 57.1,  1.6 } } },

            /* One elbow and nothing opposite it, with its corner in the top
             * rows of the frame. A lone corner that close to the top is not
             * believed whatever else the frame shows (XSEC_ELB_MIN_ROW): the
             * line it would have to be checked against leaves the frame with
             * it. The same junction is back a few frames later, lower down. */
            { "one elbow at the top of the frame (deferred)", 0, 3,
              { { 12.0, 51.6, 21.9,  2.4 },
                { 21.9,  2.4,  0.7,  7.2 },
                { 64.1, 51.6, 57.1,  1.6 } } },

            /* The same frame four rows on: a lone elbow with everything in
             * view. Accepted on the geometry alone when XSEC_ELB_SINGLE is
             * on; the expectation follows the flag so this suite is right in
             * either build. */
            { "one elbow, right line plain", XSEC_ELB_SINGLE, 3,
              { { 12.0, 55.6, 21.9,  6.4 },
                { 21.9,  6.4,  0.7, 11.2 },
                { 64.1, 55.6, 57.1,  5.6 } } },

            /* The lone elbow again, but now the camera's own detector agrees a
             * four-branch junction sits on that corner. With XSEC_ELB_SINGLE
             * off that is the tie-break the geometry cannot make for itself,
             * and the only thing that separates this from the entry above. */
            { "one elbow + camera agrees", 1, 3,
              { { 12.0, 55.6, 21.9,  6.4 },
                { 21.9,  6.4,  0.7, 11.2 },
                { 64.1, 55.6, 57.1,  5.6 } }, 4, 21.9, 6.4 },

            /* Camera says junction, but away across the frame from our corner.
             * A vote for something else is not a vote for this, or the camera
             * would be a trigger rather than a corroboration - which only
             * matters when the geometry alone is not trusted. */
            { "one elbow + camera agrees elsewhere", XSEC_ELB_SINGLE, 3,
              { { 12.0, 55.6, 21.9,  6.4 },
                { 21.9,  6.4,  0.7, 11.2 },
                { 64.1, 55.6, 57.1,  5.6 } }, 4, 70.0, 48.0 },

            /* ...and a junction the camera calls a two-branch bend is not one.
             * Only decisive when the geometry alone is not trusted. */
            { "one elbow + camera says only 2 branches", XSEC_ELB_SINGLE, 3,
              { { 12.0, 55.6, 21.9,  6.4 },
                { 21.9,  6.4,  0.7, 11.2 },
                { 64.1, 55.6, 57.1,  5.6 } }, 2, 21.9, 6.4 },

            /* A lone elbow whose far line the camera cut in two, collinear.
             * Straight is straight however many pieces it comes in. */
            { "one elbow, right line in two straight pieces", XSEC_ELB_SINGLE, 4,
              { { 12.0, 55.6, 21.9,  6.4 },
                { 21.9,  6.4,  0.7, 11.2 },
                { 64.1, 55.6, 60.0, 26.0 },
                { 60.0, 26.0, 57.1,  5.6 } } },

            /* THE LOOKALIKES OF A LONE ELBOW, typed off the simulator's own
             * frames: every one of these fired the single-elbow rule before
             * the drift, pre-bend and flatness tests existed, and between
             * them they put the car off the track on four of five circuits.
             *
             * The inside edge of a bend, seen from the straight before it.
             * The inner line turns across the top of the frame exactly like
             * a crossing edge does; what gives it away is the outer line
             * turning with it, chord by chord. */
            { "bend ahead: inside edge turns, outside follows", 0, 6,
              { {  0.0, 27.0, 19.0, 12.0 },
                { 19.0, 12.0, 25.0,  1.0 },
                { 25.0,  1.0,  0.0,  1.0 },
                { 78.0, 27.0, 64.0, 16.0 },
                { 64.0, 16.0, 53.0,  7.0 },
                { 53.0,  7.0, 38.0,  1.0 } } },

            /* Same, with the outer edge's sweep in one short top piece. */
            { "bend ahead: outside sweeps in a 3-row piece", 0, 6,
              { {  0.0, 36.0, 21.0, 14.0 },
                { 21.0, 14.0, 27.0,  2.0 },
                { 27.0,  2.0,  0.0,  3.0 },
                { 78.0, 36.0, 59.0, 16.0 },
                { 59.0, 16.0, 47.0,  4.0 },
                { 47.0,  4.0, 26.0,  1.0 } } },

            /* The mirror image, a right-hand bend. */
            { "right bend ahead, mirror of the above", 0, 6,
              { {  1.0, 33.0, 20.0, 16.0 },
                { 20.0, 16.0, 33.0,  4.0 },
                { 33.0,  4.0, 52.0,  1.0 },
                { 77.0, 38.0, 59.0, 16.0 },
                { 59.0, 16.0, 53.0,  2.0 },
                { 53.0,  2.0, 78.0,  2.0 } } },

            /* Already turning: the corridor sweeps left half a column a row
             * and the inside corner sits at the top. */
            { "bend under way, inside corner at the top", 0, 6,
              { {  1.0, 47.0, 21.0, 16.0 },
                { 21.0, 16.0, 27.0,  1.0 },
                { 27.0,  1.0,  0.0,  1.0 },
                { 78.0, 19.0, 64.0, 12.0 },
                { 64.0, 12.0, 53.0,  6.0 },
                { 53.0,  6.0, 41.0,  1.0 } } },

            /* Mid-corner: a 75 degree kink between two steep chords of the
             * inner edge. Square enough to pass the angle test - and its
             * arm climbs eight rows in five columns, which no crossing edge
             * can do, because a crossing edge lies across the road. */
            { "mid-corner kink, arm climbing steeply", 0, 6,
              { {  0.0, 16.0, 10.0, 10.0 },
                { 10.0, 10.0, 19.0,  4.0 },
                { 19.0,  4.0, 31.0,  1.0 },
                { 77.0, 49.0, 56.0, 22.0 },
                { 56.0, 22.0, 44.0,  9.0 },
                { 44.0,  9.0, 49.0,  1.0 } } },

            /* The inside edge curling into its corner with the outside edge
             * a single plain chord: the one frame where the other edge says
             * nothing, and the elbow's own line has to give it away. */
            { "inside edge curls into corner, outside plain", 0, 4,
              { {  0.0, 27.0, 19.0, 12.0 },
                { 19.0, 12.0, 25.0,  1.0 },
                { 25.0,  1.0,  0.0,  1.0 },
                { 78.0, 27.0, 40.0,  1.0 } } },
        };
        int ncase = (int)(sizeof(cases) / sizeof(cases[0]));
        int ci, k, bad = 0;

        printf("=== hand-built frames: the shapes of a crossing, and its lookalikes ===\n");
        printf("gate = road straight enough for the bar and splay tests (XSEC_MAX_HEAD),\n");
        printf("egate = ... for the elbow test (XSEC_ELB_MAX_HEAD)\n");
        printf("%-46s %5s %5s %6s %6s %6s %4s %7s %6s  %s\n",
               "frame", "gate", "egate", "hNear", "hFar", "curv", "rows",
               "square", "seen", "verdict");

        for (ci = 0; ci < ncase; ci++)
        {
            TrkSegment seg[8];
            TrackModel tm;
            XsecState  xs;
            int        f;

            for (k = 0; k < cases[ci].nseg; k++)
            {
                seg[k].x0 = (float)cases[ci].s[k][0];
                seg[k].y0 = (float)cases[ci].s[k][1];
                seg[k].x1 = (float)cases[ci].s[k][2];
                seg[k].y1 = (float)cases[ci].s[k][3];
            }

            Track_Init();
            Xsec_Init();
            /* Settled on the frame: the corridor width filter needs a moment,
             * and the confirm counter wants more than one agreeing frame. */
            for (f = 0; f < 8; f++)
            {
                Xsec_CameraHint(cases[ci].camB > 0, (float)cases[ci].camX,
                                (float)cases[ci].camY, (uint8_t)cases[ci].camB);
                Track_Update(seg, (uint8_t)cases[ci].nseg, &tm);
                Xsec_Update(seg, (uint8_t)cases[ci].nseg, &tm, 0.0f, 0.0f, 0.0f, &xs);
            }

            {
                int seen = xs.recognised ? 1 : 0;
                int ok   = (cases[ci].want == 2) ? 1 : (seen == cases[ci].want);

                if (!ok)
                {
                    bad++;
                }
                /*
                 * Whether the safety interlock even let the detectors
                 * speak. A negative case that fails this proves nothing
                 * about the detector - the frame was thrown out before it
                 * was ever asked - so for the corner frames to be worth
                 * anything their gate has to come back open.
                 */
                int gate = (tm.haveTrack &&
                            (fabsf(tm.headFar) <= XSEC_MAX_HEAD) &&
                            (fabsf(tm.headNear) <= XSEC_MAX_HEAD) &&
                            (fabsf(tm.curv) <= XSEC_MAX_CURV));
                int egate = (tm.haveTrack &&
                             (fabsf(tm.headFar) <= XSEC_ELB_MAX_HEAD) &&
                             (fabsf(tm.headNear) <= XSEC_ELB_MAX_HEAD) &&
                             (fabsf(tm.curv) <= XSEC_MAX_CURV));

                printf("%-46s %5s %5s %6.2f %6.2f %6.2f %4u %7.3f %6s  %s\n",
                       cases[ci].name, gate ? "open" : "shut", egate ? "open" : "shut",
                       tm.headNear, tm.headFar, tm.curv, tm.nValid, xs.square,
                       seen ? "yes" : "no",
                       (cases[ci].want == 2)
                         ? (seen ? "recognised" : "quiet - reported only, see below")
                         : ok ? (cases[ci].want ? "recognised, as it must be"
                                                : "quiet, as it must be")
                          : (cases[ci].want ? "*** MISSED IT ***"
                                            : "*** FALSE POSITIVE ***"));
            }
        }

        printf("\n%d of %d frames read correctly\n", ncase - bad, ncase);
        printf("\nThe 3rd PixyMon frame carries no signature any detector can use: the\n");
        printf("camera drew our two lines stopping at mid-frame and nothing along the\n");
        printf("crossing near edge. It arrives a few frames after the elbow frame, and\n");
        printf("the recognition is carried across it by XSEC_AHEAD - which -xsec\n");
        printf("exercises - not by anything here.\n");
        return bad ? 1 : 0;
    }

    if (argc > 1 && strcmp(argv[1], "-xdump") == 0)
    {
        TrackSeg t[] = { {0.0, 600.0} };
        Cam      cm;
        Result   r;

        cm.f = 68.0;
        cm.h     = (argc > 5) ? atof(argv[5]) : 18.0;
        cm.horiz = (argc > 6) ? atof(argv[6]) : -4.0;
        if (argc > 7) g_xsecMerge = atoi(argv[7]);
        g_dumpFrom = (argc > 2) ? atof(argv[2]) : 1.0;
        g_dumpTo   = (argc > 3) ? atof(argv[3]) : 3.0;

        xsec_clear();
        xsec_add(250.0, (argc > 4) ? atof(argv[4]) : 90.0, 22.5);

        printf("=== straight track, square crossing at 250 cm ===\n");
        r = run(t, 1, &cm, 22.5, 0.0, "xdump", 0, 6.0);
        report(&r);
        xsec_clear();
        return 0;
    }

    /* Intersections: recognise them and keep the power on. */
    if (argc > 1 && strcmp(argv[1], "-xsec") == 0)
    {
        struct
        {
            const char *name;
            double      ang;   /* crossing angle, degrees */
            double      xw;    /* crossing track half width, cm */
            double      halfW; /* our half width, cm */
            double      curv;  /* curvature of our track at the crossing */
            double      h;     /* camera height, cm                     */
            double      horiz; /* camera horizon row                    */
            int         merge; /* 1 = camera follows the corner round    */
            double      kick;  /* yaw step 100 cm before it, degrees     */
            double      pre;   /* an R=120 bend ending 40 cm before it   */
            int         stub;  /* 0 = both crossing stubs drawn, -1/+1 = only one */
            int         fit;   /* 1 = chords fitted to the paint, as a tracker draws */
        } k[] = {
            /* Default mounting: 18 cm mast. The near track is wider than the
             * frame here, so the two lines only appear once they are a good way
             * off - which leaves the splay test no headroom and puts the whole
             * job on the bar test. */
            { "square, 45cm track",   90.0, 22.5, 22.5, 0.0,         18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "square, 60cm track",   90.0, 30.0, 30.0, 0.0,         18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "square, narrow cross", 90.0, 17.5, 22.5, 0.0,         18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "square, wide cross",   90.0, 30.0, 22.5, 0.0,         18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "60 deg skew",          60.0, 22.5, 22.5, 0.0,         18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "45 deg skew",          45.0, 22.5, 22.5, 0.0,         18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "120 deg skew",        120.0, 22.5, 22.5, 0.0,         18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "on a gentle bend",     90.0, 22.5, 22.5, 1.0 / 220.0, 18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            { "on a tighter bend",    90.0, 22.5, 22.5, 1.0 / 120.0, 18.0, -4.0,  1, 0.0, 0.0, 0, 0 },
            /* Taller mast, in the aim envelope: both lines are inside the frame
             * with room either side, which is what a crossing needs in order to
             * be able to look wider than they do. This is the mounting the splay
             * test is for. */
            { "tall mast, square",    90.0, 22.5, 22.5, 0.0,         26.0, -10.0, 1, 0.0, 0.0, 0, 0 },
            { "tall mast, 45 skew",   45.0, 22.5, 22.5, 0.0,         26.0, -10.0, 1, 0.0, 0.0, 0, 0 },
            { "tall mast, wide cross",90.0, 30.0, 22.5, 0.0,         26.0, -10.0, 1, 0.0, 0.0, 0, 0 },
            /* Same again with the camera breaking the paint at the corner
             * instead of following it round, which some frames do. */
            { "split corner, square",  90.0, 22.5, 22.5, 0.0,        26.0, -10.0, 0, 0.0, 0.0, 0, 0 },
            { "split corner, 18cm",    90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  0, 0.0, 0.0, 0, 0 },
            /* The frame a real Pixy2 gave: the paint broken at the corner,
             * only ONE side of the crossing drawn, and every straight piece
             * of paint one vector, the way a line tracker draws it. No bars
             * on both sides, no splay - the lone elbow is the only thing
             * that can see it. */
            { "one stub only, left",   90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  0, 0.0, 0.0, -1, 1 },
            { "one stub only, right",  90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  0, 0.0, 0.0, +1, 1 },
            { "tall, one stub, left",  90.0, 22.5, 22.5, 0.0,        26.0, -10.0, 0, 0.0, 0.0, -1, 1 },
            { "tall, one stub, right", 90.0, 22.5, 22.5, 0.0,        26.0, -10.0, 0, 0.0, 0.0, +1, 1 },
            /* Arriving at the junction still correcting. The held angle is
             * sampled from a car that is steering to undo a yaw; carried
             * across the gap unchanged it goes on turning the car after the
             * yaw is gone. The real frames show exactly this - a corridor
             * sweeping 0.6 columns a row at a square crossing. */
            { "square, yawed 5 deg",   90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  1, 5.0, 0.0, 0, 0 },
            { "square, yawed 8 deg",   90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  1, 8.0, 0.0, 0, 0 },
            { "square, yawed -8 deg",  90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  1, -8.0, 0.0, 0, 0 },
            { "tall mast, yawed 8",    90.0, 22.5, 22.5, 0.0,        26.0, -10.0, 1, 8.0, 0.0, 0, 0 },
            /* The same thing the way a layout produces it: a bend ending
             * just before the crossing, so the car is still unwinding. */
            { "square, 40cm after bend",90.0, 22.5, 22.5, 0.0,       18.0, -4.0,  1, 0.0, 70.0, 0, 0 },
            { "tall, 40cm after bend", 90.0, 22.5, 22.5, 0.0,        26.0, -10.0, 1, 0.0, 70.0, 0, 0 },
            /* ...and the way the real camera draws that: one elbow. */
            { "after bend, one stub L",90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  0, 0.0, 70.0, -1, 1 },
            { "after bend, one stub R",90.0, 22.5, 22.5, 0.0,        18.0, -4.0,  0, 0.0, 70.0, +1, 1 },
            { "tall, after bend, stub L",90.0, 22.5, 22.5, 0.0,      26.0, -10.0, 0, 0.0, 70.0, -1, 1 },
        };
        int i;
        int pass  = 0;
        int total = (int)(sizeof(k) / sizeof(k[0]));
        int one   = (argc > 2) ? atoi(argv[2]) : -1;

        /*
         * Every case is run twice: once on bare track, once with the crossing
         * painted on it and nothing else changed. The question is not whether
         * the car can take this piece of track flat out - some of it bends, and
         * it should slow for that - but whether the junction costs it anything.
         * So the bare run is the reference and the crossing run is measured
         * against it, which is the only comparison that isolates the feature.
         */
        printf("=== intersections: does the junction cost anything? ===\n");
        printf("%-22s %-15s %-15s %-7s %s\n",
               "crossing", "minspd bare>x", "clearance b>x", "excurs", "verdict");

        for (i = 0; i < total; i++)
        {
            TrackSeg t[4];
            int      nt;
            Cam      cm;
            Result   ctrl, r;
            int      ok;

            /* The crossing sits at 300 cm whatever comes before it. */
            if (k[i].pre > 0.0)
            {
                t[0].curv = 0.0;         t[0].len = 260.0 - k[i].pre - 40.0;
                t[1].curv = 1.0 / 120.0; t[1].len = k[i].pre;
                t[2].curv = 0.0;         t[2].len = 40.0;
                t[3].curv = k[i].curv;   t[3].len = 340.0;
                nt = 4;
            }
            else
            {
                t[0].curv = 0.0;       t[0].len = 260.0;
                t[1].curv = k[i].curv; t[1].len = 340.0;
                nt = 2;
            }
            g_kickAt  = (k[i].kick != 0.0) ? 200.0 : -1.0;
            g_kickDeg = k[i].kick;
            g_xsecStub = k[i].stub;
            g_fitChunks = k[i].fit;

            if ((one >= 0) && (one != i)) continue;

            cm.f = 68.0; cm.h = k[i].h; cm.horiz = k[i].horiz;
            g_xsecMerge = k[i].merge;

            /* Score the stretch from where the crossing first comes into view
             * to well clear of it on the far side. */
            g_winLo = (int)(230.0 / g_step);
            g_winHi = (int)(370.0 / g_step);

            xsec_clear();
            if (one >= 0) printf("--- control: no crossing ---\n");
            ctrl = run(t, nt, &cm, k[i].halfW, 0.0, k[i].name, (one >= 0), 20.0);
            if (one >= 0) printf("--- with the crossing ---\n");

            xsec_clear();
            xsec_add(300.0, k[i].ang, k[i].xw);
            /* -xsec <case> <from> <to>: raw frames over that window of the
             * crossing run, in seconds. */
            if ((one >= 0) && (argc > 4))
            {
                g_dumpFrom = atof(argv[3]);
                g_dumpTo   = atof(argv[4]);
            }
            r = run(t, nt, &cm, k[i].halfW, 0.0, k[i].name, (one >= 0), 20.0);
            g_dumpFrom = -1.0;
            g_dumpTo   = -1.0;

            g_winLo = -1;
            g_winHi = -1;
            g_kickAt = -1.0;
            g_kickDeg = 0.0;
            g_xsecStub = 0;
            g_fitChunks = 0;
            xsec_clear();

            /*
             * The junction must not put a wheel over a line, must not stop the
             * car, must not cost it speed, and must not eat the safety margin.
             *
             * Margin, not lateral deviation. Using more of the track while
             * going faster is what a racing line IS - scoring the deviation
             * itself would mark the car down for driving well, and would push
             * any tuning built on this test toward the middle of the road.
             * What must not shrink is the distance left to the black line.
             */
            ok = r.finished && (r.excursions == 0) &&
                 (r.minSpeedWin >= (0.92 * ctrl.minSpeedWin)) &&
                 (r.minClear >= (ctrl.minClear - 3.0));

            if (ok) pass++;
            printf("%-22s %6.0f > %-6.0f %6.2f > %-6.2f %-7d %s\n",
                   k[i].name, ctrl.minSpeedWin, r.minSpeedWin,
                   ctrl.minClear, r.minClear, r.excursions,
                   !r.finished          ? "DID NOT FINISH"
                   : (r.excursions > 0) ? "LEFT THE TRACK"
                                        : (ok ? "costs nothing"
                                              : "the junction cost it"));
        }
        printf("\n%d of %d crossings cost the car nothing\n", pass, total);
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
