/*
 * telemetry.h - flight recorder.
 *
 * The car cannot drive with a cable attached, and the two obvious ways to get data
 * out are both unusable here: the debug console is semihosted, so it needs a debugger
 * and stalls the control loop for milliseconds at a time; and the debug UART is on
 * LPUART4, whose pins this board's pin_mux does not route anywhere.
 *
 * So nothing is printed. Instead one compact 32 byte record per camera frame is
 * written into a ring buffer in RAM, which costs a few dozen nanoseconds, and the
 * whole buffer is read out afterwards through the SWD probe with tools/capture.ps1.
 * Drive a few laps untethered, plug in, dump, done.
 *
 * The buffer lives in SRAMX, which this project otherwise does not use at all, in the
 * linker's no-init section - so the log survives a reset and is still there even if
 * attaching the debugger restarts the chip.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "driver.h"

#define TLM_MAGIC       0x314D4C54u /* "TLM1" */
#define TLM_VERSION     1u
#define TLM_SLOTS       3000u       /* 32 B each: ~96 KB, about 50 s at 60 fps */

/* flags bits */
#define TLM_F_HAVETRACK 0x01u
#define TLM_F_BOTHEDGES 0x02u
#define TLM_F_CHICANE   0x04u
#define TLM_F_CLAMPED   0x08u
#define TLM_F_BRAKING   0x10u
#define TLM_F_RUNNING   0x20u
#define TLM_F_BENCH     0x40u

/* Exactly 32 bytes. Fixed point, because floats would double the size for no gain
 * and the host decoder has to know the scaling anyway. */
typedef struct
{
    uint16_t frame;
    uint16_t dtUs;          /* microseconds since the previous camera frame */
    int16_t  steer_x10;
    int16_t  speed_x10;
    int16_t  targetX_x10;
    int16_t  headNear_x100;
    int16_t  headFar_x100;
    int16_t  curv_x100;
    int16_t  bias_x100;
    int16_t  widthNear_x10; /* corridor width at row 0 and at the top valid row */
    int16_t  widthFar_x10;
    int16_t  centerNear_x10;
    uint8_t  nValid;
    uint8_t  nVectors;      /* raw vectors the Pixy2 returned this frame */
    uint8_t  laRow;
    uint8_t  flags;
    uint16_t pixyErrors;
    uint16_t pixyTimeouts;
} TlmRecord;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t recordSize;
    uint32_t slots;
    uint32_t count;    /* records written since boot; wraps the ring when > slots */
    uint32_t uptimeMs;
    uint32_t reserved[2];
} TlmHeader;

typedef struct
{
    TlmHeader header;
    TlmRecord rec[TLM_SLOTS];
} TlmBuffer;

extern TlmBuffer g_tlm;

void Telemetry_Init(void);

/* Call once per camera frame, after Driver_Step. */
void Telemetry_Log(const DriveState *st,
                   const DriveCmd  *cmd,
                   uint8_t          nVectors,
                   uint32_t         dtUs,
                   uint32_t         uptimeMs,
                   uint32_t         pixyErrors,
                   uint32_t         pixyTimeouts);

#endif /* TELEMETRY_H */
