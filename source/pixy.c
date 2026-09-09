#include "pixy.h"
#include "peripherals.h"
#include "ticks.h"
#include <string.h>

/* ---- Pixy2 wire protocol ------------------------------------------------ */
#define SYNC_REQ_0  0xAEu
#define SYNC_REQ_1  0xC1u
#define SYNC_RSP_0  0xAFu
#define SYNC_RSP_1  0xC1u

#define TYPE_RESULT 0x01u  /* generic "here is your answer" reply */
#define TYPE_ERROR  0x03u  /* payload is an int32 error code, -1 means busy */

#define REQ_SET_PARAM       0xC1u  /* second sync byte doubles as the setter opcode */
#define LINE_REQ_FEATURES   0x30u
#define LINE_RSP_FEATURES   0x31u
#define LINE_REQ_SET_MODE   0x36u
#define REQ_SET_LAMP        0x16u

#define LINE_GET_ALL        0x01u  /* every vector, not just the one Pixy picked */
#define LINE_FEATURE_VECTOR 0x01u

#define HDR_LEN             6u     /* sync(2) type(1) length(1) checksum(2) */
#define PIXY_BUF_LEN        96u
#define RESYNC_TRIES        16u

#define XFER_TIMEOUT_US     6000u  /* a 100 kHz byte is 90 us; 6 ms is a dead bus */

/* ---- transport ---------------------------------------------------------- */

static void pixy_edma_cb(LPI2C_Type *base,
                         lpi2c_master_edma_handle_t *handle,
                         status_t status,
                         void *userData)
{
    pixy_t *cam = (pixy_t *)userData;
    (void)base;
    (void)handle;

    if (cam != NULL)
    {
        cam->result = status;
        cam->done   = true;
    }
}

static status_t pixy_xfer(pixy_t *cam, uint8_t *data, size_t len, bool read)
{
    lpi2c_master_transfer_t xfer;
    status_t                s;
    uint32_t                t0;

    if (len == 0u)
    {
        return kStatus_Success;
    }

    (void)memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress   = cam->address;
    xfer.direction      = read ? kLPI2C_Read : kLPI2C_Write;
    xfer.subaddressSize = 0u;
    xfer.data           = data;
    xfer.dataSize       = len;
    xfer.flags          = kLPI2C_TransferDefaultFlag;

    cam->done   = false;
    cam->result = kStatus_Success;

    s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success)
    {
        cam->errors++;
        return s;
    }

    t0 = Ticks_Us();
    while (!cam->done)
    {
        if ((Ticks_Us() - t0) > XFER_TIMEOUT_US)
        {
            (void)LPI2C_MasterTransferAbortEDMA(cam->instance, &cam->edmaHandle);
            cam->timeouts++;
            return kStatus_Timeout;
        }
    }

    if (cam->result != kStatus_Success)
    {
        cam->errors++;
        return cam->result;
    }

    return kStatus_Success;
}

static status_t pixy_send(pixy_t *cam, uint8_t opcode, const uint8_t *payload, uint8_t len)
{
    uint8_t cmd[8];

    if (len > (uint8_t)(sizeof(cmd) - 4u))
    {
        return kStatus_InvalidArgument;
    }

    cmd[0] = SYNC_REQ_0;
    cmd[1] = SYNC_REQ_1;
    cmd[2] = opcode;
    cmd[3] = len;
    if (len > 0u)
    {
        (void)memcpy(&cmd[4], payload, len);
    }

    return pixy_xfer(cam, cmd, (size_t)len + 4u, false);
}

/*
 * Reads one response header. Normally the two sync bytes land at offset 0. If they
 * do not, the bus is out of step with the packet stream, so bytes are dropped one
 * at a time until the sync word shows up rather than decoding garbage.
 */
static status_t pixy_recv_header(pixy_t *cam, uint8_t *type, uint8_t *len, uint16_t *csum)
{
    uint8_t  hdr[HDR_LEN];
    uint8_t  b;
    uint8_t  prev;
    uint32_t tries;
    status_t s;

    s = pixy_xfer(cam, hdr, HDR_LEN, true);
    if (s != kStatus_Success)
    {
        return s;
    }

    if ((hdr[0] != SYNC_RSP_0) || (hdr[1] != SYNC_RSP_1))
    {
        prev = hdr[HDR_LEN - 1u];
        for (tries = 0u; tries < RESYNC_TRIES; tries++)
        {
            s = pixy_xfer(cam, &b, 1u, true);
            if (s != kStatus_Success)
            {
                return s;
            }
            if ((prev == SYNC_RSP_0) && (b == SYNC_RSP_1))
            {
                break;
            }
            prev = b;
        }
        if (tries >= RESYNC_TRIES)
        {
            cam->errors++;
            return kStatus_Fail;
        }
        /* Sync word consumed, the remaining four header bytes follow it. */
        s = pixy_xfer(cam, &hdr[2], HDR_LEN - 2u, true);
        if (s != kStatus_Success)
        {
            return s;
        }
    }

    *type = hdr[2];
    *len  = hdr[3];
    *csum = (uint16_t)hdr[4] | ((uint16_t)hdr[5] << 8);

    return kStatus_Success;
}

static status_t pixy_recv_payload(pixy_t *cam, uint8_t *buf, uint8_t len, uint16_t csum)
{
    uint16_t sum = 0u;
    uint8_t  i;
    status_t s;

    if (len == 0u)
    {
        return (csum == 0u) ? kStatus_Success : kStatus_Fail;
    }
    if (len > PIXY_BUF_LEN)
    {
        cam->errors++;
        return kStatus_Fail;
    }

    s = pixy_xfer(cam, buf, len, true);
    if (s != kStatus_Success)
    {
        return s;
    }

    for (i = 0u; i < len; i++)
    {
        sum = (uint16_t)(sum + buf[i]);
    }
    if (sum != csum)
    {
        cam->errors++;
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/* Sends a request and swallows the acknowledgement, for commands with no useful reply. */
static status_t pixy_command(pixy_t *cam, uint8_t opcode, const uint8_t *payload, uint8_t len)
{
    uint8_t  buf[PIXY_BUF_LEN];
    uint8_t  type;
    uint8_t  rlen;
    uint16_t csum;
    status_t s;

    s = pixy_send(cam, opcode, payload, len);
    if (s != kStatus_Success)
    {
        return s;
    }

    s = pixy_recv_header(cam, &type, &rlen, &csum);
    if (s != kStatus_Success)
    {
        return s;
    }

    return pixy_recv_payload(cam, buf, rlen, csum);
}

/* ---- public API --------------------------------------------------------- */

void pixy_init(pixy_t *cam, LPI2C_Type *inst, uint8_t addr,
               edma_handle_t *rxHandle, edma_handle_t *txHandle)
{
    cam->instance = inst;
    cam->address  = addr;
    cam->done     = false;
    cam->result   = kStatus_Success;
    cam->errors   = 0u;
    cam->timeouts = 0u;

    LPI2C_MasterCreateEDMAHandle(cam->instance, &cam->edmaHandle,
                                 rxHandle, txHandle, pixy_edma_cb, cam);
}

status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t payload[3];

    payload[0] = r;
    payload[1] = g;
    payload[2] = b;

    return pixy_command(cam, 0x14u, payload, 3u);
}

status_t pixy_set_lamp(pixy_t *cam, bool upper, bool lower)
{
    uint8_t payload[2];

    payload[0] = upper ? 1u : 0u;
    payload[1] = lower ? 1u : 0u;

    return pixy_command(cam, REQ_SET_LAMP, payload, 2u);
}

status_t pixy_set_line_mode(pixy_t *cam, uint8_t mode)
{
    return pixy_command(cam, LINE_REQ_SET_MODE, &mode, 1u);
}

status_t pixy_get_vectors(pixy_t *cam, PixyVector *out, uint8_t max, uint8_t *count)
{
    uint8_t  req[2];
    uint8_t  buf[PIXY_BUF_LEN];
    uint8_t  type;
    uint8_t  len;
    uint16_t csum;
    uint8_t  idx;
    uint8_t  n = 0u;
    status_t s;

    *count = 0u;

    req[0] = LINE_GET_ALL;        /* every vector, so both track edges come back */
    req[1] = LINE_FEATURE_VECTOR; /* vectors only - intersections and barcodes are noise here */

    s = pixy_send(cam, LINE_REQ_FEATURES, req, 2u);
    if (s != kStatus_Success)
    {
        return s;
    }

    s = pixy_recv_header(cam, &type, &len, &csum);
    if (s != kStatus_Success)
    {
        return s;
    }

    s = pixy_recv_payload(cam, buf, len, csum);
    if (s != kStatus_Success)
    {
        return s;
    }

    /* TYPE_ERROR with payload -1 simply means "frame not ready", which is normal
     * whenever the loop runs faster than the camera. Not an error worth counting. */
    if (type != LINE_RSP_FEATURES)
    {
        return kStatus_NoData;
    }

    /* Payload is a chain of [featureType][featureLength][data...] blocks. */
    idx = 0u;
    while (((uint16_t)idx + 2u) <= (uint16_t)len)
    {
        uint8_t fType = buf[idx];
        uint8_t fLen  = buf[idx + 1u];
        uint8_t i;

        if (((uint16_t)idx + 2u + (uint16_t)fLen) > (uint16_t)len)
        {
            break; /* truncated block, stop rather than read past the payload */
        }

        if (fType == LINE_FEATURE_VECTOR)
        {
            const uint8_t *d = &buf[idx + 2u];
            uint8_t        nv = fLen / 6u;

            for (i = 0u; (i < nv) && (n < max); i++)
            {
                const uint8_t *v = &d[i * 6u];
                uint8_t        ax = v[0], ay = v[1], bx = v[2], by = v[3];

                /* Order the endpoints so the tail is always the end nearest the car. */
                if (ay < by)
                {
                    uint8_t t;
                    t = ax; ax = bx; bx = t;
                    t = ay; ay = by; by = t;
                }

                out[n].x0    = ax;
                out[n].y0    = ay;
                out[n].x1    = bx;
                out[n].y1    = by;
                out[n].index = v[4];
                out[n].flags = v[5];
                n++;
            }
        }

        idx = (uint8_t)(idx + 2u + fLen);
    }

    *count = n;
    return kStatus_Success;
}
