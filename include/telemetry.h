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
 * The buffer lives in a no-init section of MAIN SRAM. It must not go in SRAMX:
 * LinkServer stages its flash driver at 0x04000000 on every connect, so a buffer
 * there is destroyed by the act of plugging in to read it. Telemetry_Init keeps an
 * existing log when the header still matches, so the log genuinely does survive a
 * reset - including the one the debugger causes when it attaches.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "driver.h"

#define TLM_MAGIC       0x314D4C54u /* "TLM1" */
#define TLM_VERSION     2u
#define TLM_SLOTS       3000u       /* 32 B each: ~96 KB, about 50 s at 60 fps */

/* flags bits */
#define TLM_F_HAVETRACK 0x01u
#define TLM_F_BOTHEDGES 0x02u
#define TLM_F_CHICANE   0x04u
#define TLM_F_CLAMPED   0x08u
#define TLM_F_BRAKING   0x10u
#define TLM_F_RUNNING   0x20u
#define TLM_F_BENCH     0x40u
/* The crossing latch is engaged: the car is driving a junction on a held
 * steering angle rather than on anything it can currently see. The one state in
 * the whole firmware where the wheel is not following the road, so it is worth
 * the last free bit - on a capture it marks exactly where that happened. */
#define TLM_F_XSEC      0x80u

/* Every flags bit is spoken for, so this one rides in the top bit of nValid -
 * which counts corridor rows and never exceeds TRK_ROWS, so four bits of it are
 * dead space. Mask nValid with 0x0F before reading it as a count.
 *
 * It marks the frames where the Pixy2's OWN junction detector agreed with a
 * corner we found, which is the one thing about the camera half of this that
 * cannot be checked in the simulator. Capture a few laps and compare it against
 * TLM_F_XSEC: if the camera lights up where the junctions really are and stays
 * dark elsewhere, XSEC_CAM_NEAR and the PixyMon settings are right. */
#define TLM_NV_CAMXSEC  0x80u

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
    uint8_t  nValid;        /* low nibble; bit 7 is TLM_NV_CAMXSEC */
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
