#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

//Protocol
#define LINK_SOF                0xA5u
#define LINK_STATUS_CMD         0x5Au

#define LINK_STATUS_NONE        0x00u
#define LINK_STATUS_ACK         0x06u
#define LINK_STATUS_NACK_CRC    0x15u
#define LINK_STATUS_NACK_SEQ    0x16u
#define LINK_STATUS_NACK_BUSY   0x17u
#define LINK_STATUS_NACK_FORMAT 0x18u

#define LINK_MAX_PAYLOAD        24u

/*
 * MPU transmit queue.
 *
 * Increase only if SRAM permits it.
 *
 * ATtiny84A has 512 bytes SRAM, so don't make this enormous.
 */
#define LINK_TX_QUEUE_SIZE      8u

/*
 * GU command queue.
 */
#define GU_INSTRUCTION_QUEUE_SIZE  4u

/*
 * Number of retransmissions after the initial transmission.
 */
#define LINK_MAX_RETRIES        5u

/*
 * Time between sending DATA and expecting GU to make
 * a STATUS transaction available.
 *
 * Increase this if your VGA workload can keep GU busy
 * for longer.
 */
#define LINK_ACK_TIMEOUT_MS     10u

typedef struct
{
    uint8_t seq;
    uint8_t len;
    uint8_t type;
    uint8_t data[LINK_MAX_PAYLOAD];
} LinkPacket;

/* ---------------- CRC ---------------- */

/*
 * CRC-16/XMODEM
 *
 * Polynomial: 0x1021
 * Init:       0x0000
 *
 * The same implementation must be used on both devices.
 */
static inline uint16_t crc16_update(uint16_t crc,
                                         uint8_t data)
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

static inline uint16_t calc_packet_crc(const LinkPacket *p)
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