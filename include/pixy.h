/*
 * pixy.h - Pixy2 line tracking over I2C.
 *
 * Differences from the first version of this driver, all of which matter at speed:
 *
 *  - the response header is read first and only the bytes that actually exist are
 *    read after it. The old code always pulled 100 bytes, which at 100 kHz costs
 *    9 ms per frame and capped the control loop at roughly 100 Hz.
 *  - the sync word, the packet type and the payload checksum are all verified, and
 *    the driver resynchronises instead of parsing whatever happens to be in RAM.
 *  - a busy reply (the camera has no new frame yet) is reported as "no new data"
 *    rather than being decoded as vectors.
 *  - every transfer has a timeout. A stuck I2C bus used to spin forever inside the
 *    driver, which on a moving car means no steering and no braking.
 */
#ifndef PIXY_H_
#define PIXY_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "fsl_common.h"
#include "fsl_lpi2c_edma.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIXY_MAX_VECTORS 12u

/* Line tracker modes, OR them together for pixy_set_line_mode(). */
#define PIXY_LINE_MODE_TURN_DELAYED     0x01u
#define PIXY_LINE_MODE_MANUAL_SELECT    0x02u
#define PIXY_LINE_MODE_WHITE_LINE       0x80u

/* One line segment on the 79 x 52 line tracking grid.
 * The driver orders the endpoints so that (x0,y0) is the tail, nearest the car
 * (larger y), and (x1,y1) is the head, farther down the track (smaller y). */
typedef struct
{
    uint8_t x0, y0;
    uint8_t x1, y1;
    uint8_t index;
    uint8_t flags;
} PixyVector;

typedef struct
{
    LPI2C_Type                *instance;
    uint8_t                    address;
    lpi2c_master_edma_handle_t edmaHandle;
    volatile bool              done;
    volatile status_t          result;
    uint32_t                   errors;   /* running count, useful while debugging */
    uint32_t                   timeouts;
} pixy_t;

void     pixy_init(pixy_t *cam, LPI2C_Type *inst, uint8_t addr,
                   edma_handle_t *rxHandle, edma_handle_t *txHandle);

status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b);
status_t pixy_set_lamp(pixy_t *cam, bool upper, bool lower);
status_t pixy_set_line_mode(pixy_t *cam, uint8_t mode);

/* Fetches every line vector in the current frame.
 * Returns kStatus_Success when a frame was decoded - *count may legitimately be 0
 * if the camera sees no line. Any other return means there is no new data this
 * call (camera still busy, bad packet, or bus timeout); the caller should keep
 * using the previous frame and start counting how long it has been blind. */
status_t pixy_get_vectors(pixy_t *cam, PixyVector *out, uint8_t max, uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* PIXY_H_ */
