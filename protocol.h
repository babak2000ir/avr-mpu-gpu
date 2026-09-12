#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* ============================================================
 * Protocol Control & Status Constants
 * ============================================================ */

#define LINK_SOF                0xA5u

/*
 * The wire no longer carries a separate status-poll SPI transaction.
 *
 * The current contract is:
 *   - GU drives READY to indicate the result is valid or that another
 *     DATA packet may be sent.
 *   - On the same moment READY rises, MPU samples RESULT.
 *   - RESULT=0 => accept, RESULT=1 => reject.
 *
 * This removes one entire CS transaction per command and keeps the
 * protocol path out of the VGA timing ISR.
 *
 * The codes below remain as GU-local reject reasons for diagnostics
 * (see GU_LastRejectReason()). They are not transmitted over the link.
 */
#define LINK_STATUS_NONE        0x00u
#define LINK_STATUS_ACK         0x06u
#define LINK_STATUS_NACK_CRC    0x15u
#define LINK_STATUS_NACK_SEQ    0x16u
#define LINK_STATUS_NACK_BUSY   0x17u
#define LINK_STATUS_NACK_FORMAT 0x18u

#define LINK_MAX_PAYLOAD        24u

/*
 * MPU transmit queue size.
 */
#define LINK_TX_QUEUE_SIZE      8u

/*
 * GU command queue size.
 */
#define GU_INSTRUCTION_QUEUE_SIZE  4u

/*
 * Number of retransmissions after the initial transmission.
 */
#define LINK_MAX_RETRIES        5u

/*
 * Time between sending DATA and expecting GU to raise READY
 * with the result of that transaction.
 */
#define LINK_ACK_TIMEOUT_MS     10u

/*
 * Once LINK_MAX_RETRIES fast retries have been exhausted for a
 * packet, MPU stops hammering the link and backs off to this
 * period between attempts instead. It NEVER gives up on the
 * packet outright (that would desync the sequence numbers with
 * GU permanently) -- it just retries more slowly so a jammed
 * or reset GU has room to recover without the MPU spinning.
 */
#define LINK_BACKOFF_MS          250u

/* ============================================================
 * Graphic Command Types & Constants
 * ============================================================ */

#define CMD_SET_PIXEL           0x01u
#define CMD_SET_LINE            0x02u
#define CMD_SET_RECT            0x03u
#define CMD_SET_CIRCLE          0x04u
#define CMD_SET_STRING          0x05u

/* Color packing/unpacking helpers (4 bits border / low, 4 bits fill / high) */
#define MAKE_COLOR_BF(border, fill)  (((uint8_t)((fill) & 0x0Fu) << 4) | ((uint8_t)(border) & 0x0Fu))
#define COLOR_GET_BORDER(c)          ((uint8_t)((c) & 0x0Fu))
#define COLOR_GET_FILL(c)            ((uint8_t)(((c) >> 4) & 0x0Fu))

/* ============================================================
 * Shared Packet & Command Structures
 * ============================================================ */

typedef struct
{
    uint8_t seq;
    uint8_t len;
    uint8_t type;
    uint8_t data[LINK_MAX_PAYLOAD];
} LinkPacket;

typedef struct
{
    uint8_t seq;
    uint8_t type;
    uint8_t len;
    uint8_t data[LINK_MAX_PAYLOAD];
} GuCommand;

/* ============================================================
 * Graphic Command Structures & Parser Helpers
 * ============================================================ */

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint8_t color; /* 4 bits used */
} CmdSetPixel;

typedef struct
{
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    uint8_t color; /* 4 bits used */
} CmdSetLine;

typedef struct
{
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    uint8_t color; /* 4 bits border, 4 bits fill */
} CmdSetRect;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t radius;
    uint8_t color; /* 4 bits border, 4 bits fill */
} CmdSetCircle;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint8_t color; /* 4 bits used */
    const char *text; /* null-terminated string */
} CmdSetString;

static inline bool GU_ParseSetPixel(const uint8_t *data, uint8_t len, CmdSetPixel *cmd)
{
    if (!data || len < 5 || !cmd) return false;
    cmd->x = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    cmd->y = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    cmd->color = data[4] & 0x0Fu;
    return true;
}

static inline bool GU_ParseSetLine(const uint8_t *data, uint8_t len, CmdSetLine *cmd)
{
    if (!data || len < 9 || !cmd) return false;
    cmd->x1 = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    cmd->y1 = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    cmd->x2 = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    cmd->y2 = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
    cmd->color = data[8] & 0x0Fu;
    return true;
}

static inline bool GU_ParseSetRect(const uint8_t *data, uint8_t len, CmdSetRect *cmd)
{
    if (!data || len < 9 || !cmd) return false;
    cmd->x1 = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    cmd->y1 = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    cmd->x2 = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    cmd->y2 = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
    cmd->color = data[8];
    return true;
}

static inline bool GU_ParseSetCircle(const uint8_t *data, uint8_t len, CmdSetCircle *cmd)
{
    if (!data || len < 7 || !cmd) return false;
    cmd->x = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    cmd->y = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    cmd->radius = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    cmd->color = data[6];
    return true;
}

static inline bool GU_ParseSetString(const uint8_t *data, uint8_t len, CmdSetString *cmd)
{
    if (!data || len < 6 || !cmd) return false;
    cmd->x = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    cmd->y = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    cmd->color = data[4] & 0x0Fu;
    cmd->text = (const char *)&data[5];
    return true;
}

/* ============================================================
 * Shared CRC-16/XMODEM Calculation
 * ============================================================ */

static inline uint16_t crc16_update(uint16_t crc, uint8_t data)
{
    crc ^= ((uint16_t)data << 8);

    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x8000u)
            crc = (uint16_t)((crc << 1) ^ 0x1021u);
        else
            crc <<= 1;
    }

    return crc;
}

static inline uint16_t link_packet_crc(const LinkPacket *p)
{
    uint16_t crc = 0;

    crc = crc16_update(crc, p->seq);
    crc = crc16_update(crc, p->len);
    crc = crc16_update(crc, p->type);

    for (uint8_t i = 0; i < p->len; i++)
        crc = crc16_update(crc, p->data[i]);

    return crc;
}

#endif