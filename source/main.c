/*
 * NXP Cup - racing firmware.
 *
 * One loop, four steps:
 *
 *   1. ask the Pixy2 for the line segments in the current frame
 *   2. track.c   works out where the two black lines are, at eight distances ahead
 *   3. racing_line.c picks the point to aim at: wide, apex, wide - then forces that
 *                    point back inside the black lines if it has to
 *   4. driver.c  converts it into a servo angle and two motor commands, and decides
 *                how fast the car is allowed to be right now
 *
 * Every number worth changing lives in include/race_config.h.
 */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_common.h"

#include "race_config.h"
#include "ticks.h"
#include "pixy.h"
#include "track.h"
#include "racing_line.h"
#include "driver.h"
#include "intersection.h"
#include "hbridge.h"
#include "servo.h"
#if RACE_TELEMETRY
#include "telemetry.h"
#endif

static PixyVector s_vectors[PIXY_MAX_VECTORS];
static TrkSegment s_segments[PIXY_MAX_VECTORS];

static void vectors_to_segments(const PixyVector *v, uint8_t n, TrkSegment *s)
{
    uint8_t i;

    for (i = 0u; i < n; i++)
    {
        s[i].x0 = (float)v[i].x0;
        s[i].y0 = (float)v[i].y0;
        s[i].x1 = (float)v[i].x1;
        s[i].y1 = (float)v[i].y1;
    }
}

#if RACE_DEBUG
static void debug_report(const DriveCmd *cmd)
{
    static uint32_t next;
    const DriveState *st = Driver_State();

    if (st->frames < next)
    {
        return;
    }
    next = st->frames + 25u;

    PRINTF("rows=%d both=%d hN=%d hF=%d tgt=%d steer=%d spd=%d sev=%d %s%s\r\n",
           (int)st->track.nValid,
           (int)st->track.bothEdges,
           (int)(st->track.headNear * 100.0f),
           (int)(st->track.headFar * 100.0f),
           (int)st->line.targetX,
           (int)cmd->steer,
           (int)cmd->speed,
           (int)(st->severity * 100.0f),
           st->line.chicane ? "CHICANE " : "",
           cmd->braking ? "BRAKE" : "");
}
#endif

int main(void)
{
    pixy_t   cam;
    DriveCmd cmd;
    uint32_t tPrev;
#if RACE_TELEMETRY
    uint32_t tFrame;
#endif

    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    /* Timebase first: the Pixy driver uses it for its transfer timeouts. */
    Ticks_Init(SystemCoreClock);

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);

    /* Wheels stopped and pointing straight before anything else happens. */
    HbridgeSpeedF(&g_hbridge, 0.0f, 0.0f);
    Steer(STEER_OFFSET);

    pixy_init(&cam, LP_FLEXCOMM2_PERIPHERAL, PIXY_I2C_ADDR,
              &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);

    /* Dim green: the camera is alive and the firmware got this far. Ignore the
     * result, a car that cannot set an LED can still race. */
    (void)pixy_set_led(&cam, 0u, 40u, 0u);

    Driver_Init();
#if RACE_TELEMETRY
    Telemetry_Init();
#endif

    tPrev = Ticks_Us();
#if RACE_TELEMETRY
    tFrame = tPrev;
#endif

    for (;;)
    {
        uint8_t  n = 0u;
        bool     fresh;
        uint32_t now;
        float    dt;

        fresh = (pixy_get_vectors(&cam, s_vectors, PIXY_MAX_VECTORS, &n) == kStatus_Success);
        if (fresh)
        {
            vectors_to_segments(s_vectors, n, s_segments);
        }

        /*
         * Hand over the camera's own junction verdict for this frame.
         *
         * Unconditionally, and false when the frame did not arrive: a stale
         * "yes" is the one way this could vouch for a corner it never saw,
         * so a dropped frame has to clear it rather than leave it standing.
         * pixy_get_vectors zeroes interCount at entry, so a failed read
         * already reports none - this just makes that explicit.
         */
        Xsec_CameraHint(fresh && (cam.interCount > 0u),
                        (float)cam.interX, (float)cam.interY,
                        cam.interBranches);

        now   = Ticks_Us();
        dt    = (float)(now - tPrev) * 1.0e-6f;
        tPrev = now;

        Driver_Step(fresh, s_segments, n, dt, &cmd);

        Steer(cmd.steer + STEER_OFFSET);
        HbridgeSpeedF(&g_hbridge, cmd.left, cmd.right);

#if RACE_TELEMETRY
        /* One record per camera frame, not per loop: the loop runs several times
         * faster than the Pixy2 and the extra records would say nothing new. */
        if (fresh)
        {
            Telemetry_Log(Driver_State(), &cmd, n, now - tFrame, now / 1000u,
                          cam.errors, cam.timeouts);
            tFrame = now;
        }
#endif

#if RACE_DEBUG
        debug_report(&cmd);
#endif
    }
}
