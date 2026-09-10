#include "telemetry.h"
#include "race_config.h"

/* Main SRAM, no-init.
 *
 * NOT SRAMX. LinkServer stages its flash driver (binaries/Flash/MCXNxxx.cfx) at
 * 0x04000000 during chip setup - it is an ELF linked to load AND execute there -
 * so a buffer in SRAMX is overwritten by the act of connecting to read it. Main
 * SRAM is untouched by the probe; at ~94 KB this still leaves over 280 KB free
 * below the stack at 0x2005F000. */
TlmBuffer g_tlm __attribute__((section(".noinit.$SRAM")));

static int16_t sat16(float v)
{
    if (v > 32767.0f)
    {
        return 32767;
    }
    if (v < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)v;
}

void Telemetry_Init(void)
{
    /* Only wipe the log when what is in RAM is not one this build can read.
     * Clearing unconditionally - which is what this did - meant every reset threw
     * the drive away, including the reset the debugger causes when it attaches to
     * read it. That contradicted the promise in telemetry.h that the log survives.
     *
     * reserved[0] counts boots, so a dump spanning a reset can still be split. */
    if ((g_tlm.header.magic == TLM_MAGIC) &&
        (g_tlm.header.version == (uint16_t)TLM_VERSION) &&
        (g_tlm.header.recordSize == (uint16_t)sizeof(TlmRecord)) &&
        (g_tlm.header.slots == TLM_SLOTS))
    {
        g_tlm.header.reserved[0]++;
        return;
    }

    g_tlm.header.magic       = TLM_MAGIC;
    g_tlm.header.version     = (uint16_t)TLM_VERSION;
    g_tlm.header.recordSize  = (uint16_t)sizeof(TlmRecord);
    g_tlm.header.slots       = TLM_SLOTS;
    g_tlm.header.count       = 0u;
    g_tlm.header.uptimeMs    = 0u;
    g_tlm.header.reserved[0] = 1u;
    g_tlm.header.reserved[1] = 0u;
}

void Telemetry_Log(const DriveState *st,
                   const DriveCmd  *cmd,
                   uint8_t          nVectors,
                   uint32_t         dtUs,
                   uint32_t         uptimeMs,
                   uint32_t         pixyErrors,
                   uint32_t         pixyTimeouts)
{
    TlmRecord *r = &g_tlm.rec[g_tlm.header.count % TLM_SLOTS];
    uint8_t    f = 0u;
    uint8_t    top;

    top = st->track.topRow;

    r->frame = (uint16_t)st->frames;
    r->dtUs  = (dtUs > 65535u) ? 65535u : (uint16_t)dtUs;

    r->steer_x10   = sat16(cmd->steer * 10.0f);
    r->speed_x10   = sat16(cmd->speed * 10.0f);
    r->targetX_x10 = sat16(st->line.targetX * 10.0f);

    r->headNear_x100 = sat16(st->track.headNear * 100.0f);
    r->headFar_x100  = sat16(st->track.headFar * 100.0f);
    r->curv_x100     = sat16(st->track.curv * 100.0f);
    r->bias_x100     = sat16(st->line.bias * 100.0f);

    r->widthNear_x10  = sat16(st->track.width[0] * 10.0f);
    r->widthFar_x10   = sat16(st->track.width[top] * 10.0f);
    r->centerNear_x10 = sat16(st->track.center[0] * 10.0f);

    r->nValid   = st->track.nValid;
    r->nVectors = nVectors;
    r->laRow    = st->line.laRow;

    if (st->track.haveTrack)
    {
        f |= TLM_F_HAVETRACK;
    }
    if (st->track.bothEdges)
    {
        f |= TLM_F_BOTHEDGES;
    }
    if (st->line.chicane)
    {
        f |= TLM_F_CHICANE;
    }
    if (st->line.clamped)
    {
        f |= TLM_F_CLAMPED;
    }
    if (cmd->braking)
    {
        f |= TLM_F_BRAKING;
    }
    if (st->elapsedMs >= START_DELAY_MS)
    {
        f |= TLM_F_RUNNING;
    }
#if RACE_BENCH_MODE
    f |= TLM_F_BENCH;
#endif

#if XSEC_ENABLE
    if (st->xsec.phase == XSEC_CROSSING)
    {
        f |= TLM_F_XSEC;
    }
#endif

#if XSEC_ENABLE
    if (st->xsec.camAgreed)
    {
        r->nValid |= TLM_NV_CAMXSEC;
    }
#endif
    r->flags = f;

    r->pixyErrors   = (pixyErrors > 65535u) ? 65535u : (uint16_t)pixyErrors;
    r->pixyTimeouts = (pixyTimeouts > 65535u) ? 65535u : (uint16_t)pixyTimeouts;

    g_tlm.header.uptimeMs = uptimeMs;
    g_tlm.header.count++;
}
