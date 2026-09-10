/*
 * mpu.c
 * ATmega328P
 * MPU = SPI master
 * GU  = SPI slave
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef F_CPU
#define F_CPU 20000000UL
#endif
#include <util/delay.h>

#include "protocol.h"
#include "mpu.h"


/* ============================================================
 * Pin configuration
 * ============================================================ */

/*
 * ATmega328P hardware SPI pins (PORTB):
 *
 * PB3 = MOSI
 * PB4 = MISO
 * PB5 = SCK
 * PB2 = SS   (manual chip-select output to the GU)
 */

#define SPI_MOSI_BIT     PB3
#define SPI_MISO_BIT     PB4
#define SPI_SCK_BIT      PB5

#define CS_PORT          PORTB
#define CS_DDR           DDRB
#define CS_BIT           PB2

#define READY_PORT       PORTD
#define READY_DDR        DDRD
#define READY_PIN_REG    PIND
#define READY_PIN        PD2

/*
 * RESULT input: accept/reject of the last DATA transaction,
 * valid the instant READY reads high. Replaces the old SPI
 * "status poll" sub-transaction -- see protocol.h.
 *
 * PB6 (XTAL1) is the external clock input and must not be used
 * as GPIO. The board is clocked externally, so PB7 (XTAL2) is
 * free for GPIO use -- used here for RESULT.
 */
#define RESULT_PORT      PORTB
#define RESULT_DDR       DDRB
#define RESULT_PIN_REG   PINB
#define RESULT_PIN       PB7


/* ============================================================
 * Configuration
 * ============================================================ */

/*
 * Hardware SPI clock divider (relative to F_CPU).
 *
 * At F_CPU = 20 MHz, SPI2X=0 and SPR[1:0]=00 gives:
 *
 *      SCK = F_CPU / 4 = 5 MHz
 *
 * IMPORTANT:
 *
 * The GU must be able to service SPI STC interrupts before the
 * next byte arrives. If your VGA ISR is long, choose a slower
 * divider (SPR bits) below.
 */


/* ============================================================
 * Time base
 * ============================================================ */

volatile uint32_t mpu_millis = 0;


/*
 * Timer0 CTC configuration for 1ms system tick at F_CPU = 20 MHz.
 *
 * 20 MHz / 256 = 78,125 Hz
 * OCR0A = 77 (78 timer counts) => 0.9984 ms (~0.16% error)
 *
 * If your MPU already has a system tick, delete this timer
 * and use your existing millisecond counter instead.
 */
static void mpu_timer_init(void)
{
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS02);

    OCR0A = 77;

    TIFR0 = _BV(OCF0A);
    TIMSK0 = _BV(OCIE0A);
}

void MPU_TimerTick(void)
{
    mpu_millis++;
}

/* ============================================================
 * Helpers
 * ============================================================ */

static inline uint32_t mpu_now_ms(void)
{
    uint32_t t;

    uint8_t sreg = SREG;
    cli();

    t = mpu_millis;

    SREG = sreg;

    return t;
}


static inline bool gu_ready(void)
{
    return (READY_PIN_REG & _BV(READY_PIN)) != 0;
}


/*
 * Only meaningful once gu_ready() reads true -- see protocol.h
 * for the ordering guarantee this relies on.
 */
static inline bool gu_result_is_reject(void)
{
    return (RESULT_PIN_REG & _BV(RESULT_PIN)) != 0;
}


static inline void cs_low(void)
{
    CS_PORT &= ~_BV(CS_BIT);
}


static inline void cs_high(void)
{
    CS_PORT |= _BV(CS_BIT);
}


/* ============================================================
 * Hardware SPI master
 * ============================================================ */

/*
 * Blocking single-byte transfer using the ATmega328P hardware
 * SPI peripheral (SPCR/SPDR/SPSR).
 */
static uint8_t spi_master_transfer(uint8_t value)
{
    SPDR = value;

    while (!(SPSR & _BV(SPIF)))
        ;

    return SPDR;
}


/* ============================================================
 * SPI initialization
 * ============================================================ */

static void mpu_spi_init(void)
{
    /*
     * MOSI, SCK and CS (manual chip-select) outputs.
     * MISO input.
     *
     * The hardware SS pin (PB2 / CS_BIT) must be driven as an
     * output, idle high, or the SPI peripheral can fall back to
     * slave mode as soon as it goes low as an input.
     */
    DDRB |= _BV(SPI_MOSI_BIT);
    DDRB |= _BV(SPI_SCK_BIT);

    DDRB &= ~_BV(SPI_MISO_BIT);

    CS_DDR |= _BV(CS_BIT);
    CS_PORT |= _BV(CS_BIT);

    /*
     * READY input.
     *
     * No internal pull-up.
     *
     * Use an external pull-down on the GU READY line.
     */
    READY_DDR &= ~_BV(READY_PIN);
    READY_PORT &= ~_BV(READY_PIN);

    /*
     * RESULT input.
     *
     * No internal pull-up -- use an external pull-down on the
     * GU RESULT line, matching READY.
     */
    RESULT_DDR &= ~_BV(RESULT_PIN);
    RESULT_PORT &= ~_BV(RESULT_PIN);

    /*
     * Master mode, SPI enabled, clock = F_CPU / 4.
     */
    SPCR = _BV(SPE) | _BV(MSTR);
    SPSR = 0;
}


/* ============================================================
 * TX queue
 * ============================================================ */

static LinkPacket tx_queue[LINK_TX_QUEUE_SIZE];

static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;
static uint8_t tx_count = 0;

static uint8_t next_sequence = 0;


static bool queue_push(uint8_t type,
                       const uint8_t *data,
                       uint8_t len)
{
    if (len > LINK_MAX_PAYLOAD)
        return false;

    if (tx_count >= LINK_TX_QUEUE_SIZE)
        return false;

    LinkPacket *p = &tx_queue[tx_tail];

    p->seq  = next_sequence++;
    p->len  = len;
    p->type = type;

    for (uint8_t i = 0; i < len; i++)
        p->data[i] = data[i];

    tx_tail++;

    if (tx_tail >= LINK_TX_QUEUE_SIZE)
        tx_tail = 0;

    tx_count++;

    return true;
}


static LinkPacket *queue_head(void)
{
    if (tx_count == 0)
        return NULL;

    return &tx_queue[tx_head];
}


static void queue_pop(void)
{
    if (tx_count == 0)
        return;

    tx_head++;

    if (tx_head >= LINK_TX_QUEUE_SIZE)
        tx_head = 0;

    tx_count--;
}


/* ============================================================
 * Public send API
 * ============================================================ */

/*
 * Call this from the application.
 *
 * IMPORTANT:
 *
 * This function does NOT wait for GU.
 *
 * It only places the packet in the transmit queue.
 */
bool MPU_Send(uint8_t type,
             const uint8_t *data,
             uint8_t len)
{
    return queue_push(type, data, len);
}


/* ============================================================
 * Current transmission state
 * ============================================================ */

typedef enum
{
    MPU_LINK_IDLE = 0,
    MPU_LINK_WAIT_RESULT,
    MPU_LINK_RETRY_WAIT,
    MPU_LINK_BACKOFF
} MpuLinkState;

static MpuLinkState link_state = MPU_LINK_IDLE;

static uint8_t retry_count = 0;

static uint32_t status_deadline = 0;


/* ============================================================
 * Send DATA packet
 * ============================================================ */

static void send_data_packet(const LinkPacket *p)
{
    uint16_t crc;

    cs_low();

    /*
     * Give the GU a little setup time after CS.
     *
     * This is intentionally tiny.
     *
     * It also gives the GU's CS pin-change ISR time to reset
     * the SPI transaction state.
     */
    _delay_us(2);

    spi_master_transfer(LINK_SOF);

    spi_master_transfer(p->seq);
    spi_master_transfer(p->len);
    spi_master_transfer(p->type);

    for (uint8_t i = 0; i < p->len; i++)
        spi_master_transfer(p->data[i]);

    crc = link_packet_crc(p);

    spi_master_transfer((uint8_t)(crc & 0xFF));
    spi_master_transfer((uint8_t)(crc >> 8));

    cs_high();
}


/* ============================================================
 * Communication service
 * ============================================================ */

/*
 * Give up on the fast retry burst and drop into a slow, bounded
 * cooldown instead of freezing.
 *
 * Deliberately does NOT touch the queue or the packet's sequence
 * number: the same packet is retried again after the backoff
 * period, forever, at a rate the link can't be hurt by. This is
 * what makes the state machine unable to get permanently stuck --
 * every path always either makes progress or schedules a future
 * retry.
 */
static void enter_backoff(uint32_t now)
{
    retry_count = 0;
    status_deadline = now + LINK_BACKOFF_MS;
    link_state = MPU_LINK_BACKOFF;
}


static void enter_retry_or_backoff(uint32_t now)
{
    if (retry_count < LINK_MAX_RETRIES)
    {
        retry_count++;
        link_state = MPU_LINK_RETRY_WAIT;
    }
    else
    {
        enter_backoff(now);
    }
}


/*
 * Call this VERY frequently from the MPU main loop.
 *
 * It never waits for READY.
 * It never waits for an ACK.
 *
 * The only bounded blocking operation is the actual SPI
 * byte transfer, which at 5 MHz is only ~1.6 us/byte.
 */
void MPU_Service(void)
{
    LinkPacket *p;
    uint32_t now;

    now = mpu_now_ms();

    switch (link_state)
    {
        case MPU_LINK_IDLE:

            p = queue_head();

            if (p == NULL)
                return;

            /*
             * GU is busy.
             *
             * Do nothing.
             *
             * The packet remains in the queue.
             */
            if (!gu_ready())
                return;

            /*
             * Send oldest packet.
             */
            send_data_packet(p);

            retry_count = 0;

            status_deadline = now + LINK_ACK_TIMEOUT_MS;

            link_state = MPU_LINK_WAIT_RESULT;

            return;


        case MPU_LINK_WAIT_RESULT:

            /*
             * GU raises READY the instant RESULT is valid for
             * the packet we just sent -- no separate status
             * transaction needed, just read the pin.
             */
            if (gu_ready())
            {
                p = queue_head();

                if (p == NULL)
                {
                    link_state = MPU_LINK_IDLE;
                    return;
                }

                if (gu_result_is_reject())
                {
                    /*
                     * CRC, format, sequence, or busy rejection --
                     * MPU doesn't need to know which. All of them
                     * mean the same thing: retransmit this exact
                     * packet, unchanged, and do not pop it.
                     */
                    enter_retry_or_backoff(now);

                    return;
                }

                /*
                 * Accepted. NOW, and only now, remove it.
                 */
                queue_pop();

                retry_count = 0;

                link_state = MPU_LINK_IDLE;

                return;
            }

            /*
             * GU hasn't raised READY yet.
             *
             * Do NOT block -- but if it takes too long (GU
             * wedged, reset, or the READY edge was somehow
             * missed), the result may simply have been lost.
             * We deliberately do NOT remove the packet.
             */
            if ((int32_t)(now - status_deadline) >= 0)
            {
                enter_retry_or_backoff(now);
            }

            return;


        case MPU_LINK_RETRY_WAIT:

            /*
             * Wait for GU to become available.
             *
             * Again: no blocking.
             */
            if (!gu_ready())
                return;

            p = queue_head();

            if (p == NULL)
            {
                link_state = MPU_LINK_IDLE;
                return;
            }

            /*
             * Retransmit EXACTLY the same sequence number.
             */
            send_data_packet(p);

            status_deadline = now + LINK_ACK_TIMEOUT_MS;

            link_state = MPU_LINK_WAIT_RESULT;

            return;


        case MPU_LINK_BACKOFF:

            /*
             * Slow cooldown after a burst of fast retries failed.
             * Never gives up outright -- just waits longer between
             * attempts so a jammed/reset GU has room to recover.
             */
            if ((int32_t)(now - status_deadline) < 0)
                return;

            if (!gu_ready())
            {
                /*
                 * Still not back. Keep waiting at the same slow
                 * rate rather than spinning.
                 */
                status_deadline = now + LINK_BACKOFF_MS;
                return;
            }

            p = queue_head();

            if (p == NULL)
            {
                link_state = MPU_LINK_IDLE;
                return;
            }

            send_data_packet(p);

            status_deadline = now + LINK_ACK_TIMEOUT_MS;

            link_state = MPU_LINK_WAIT_RESULT;

            return;
    }
}


/* ============================================================
 * Optional status helpers
 * ============================================================ */

bool MPU_IsBusy(void)
{
    return link_state != MPU_LINK_IDLE;
}


uint8_t MPU_QueuedPackets(void)
{
    return tx_count;
}


uint8_t MPU_RetryCount(void)
{
    return retry_count;
}


/*
 * True once the head-of-queue packet has exhausted a fast-retry
 * burst and dropped into the slow backoff cooldown. Link is not
 * dead -- it will keep trying -- but something is wrong and the
 * application may want to surface that (diagnostic LED, log, etc).
 */
bool MPU_LinkDegraded(void)
{
    return link_state == MPU_LINK_BACKOFF;
}


/* ============================================================
 * MPU initialization
 * ============================================================ */

void MPU_Init(void)
{
    mpu_spi_init();

    mpu_timer_init();

    link_state = MPU_LINK_IDLE;

    retry_count = 0;

    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;

    next_sequence = 0;

    sei();
}


/* ============================================================
 * Graphics Command Helpers
 * ============================================================ */

bool MPU_SendSetPixel(uint16_t x, uint16_t y, uint8_t color)
{
    uint8_t payload[5];

    payload[0] = (uint8_t)(x & 0xFFu);
    payload[1] = (uint8_t)(x >> 8);
    payload[2] = (uint8_t)(y & 0xFFu);
    payload[3] = (uint8_t)(y >> 8);
    payload[4] = color & 0x0Fu;

    return MPU_Send(CMD_SET_PIXEL, payload, sizeof(payload));
}


bool MPU_SendSetLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t color)
{
    uint8_t payload[9];

    payload[0] = (uint8_t)(x1 & 0xFFu);
    payload[1] = (uint8_t)(x1 >> 8);
    payload[2] = (uint8_t)(y1 & 0xFFu);
    payload[3] = (uint8_t)(y1 >> 8);
    payload[4] = (uint8_t)(x2 & 0xFFu);
    payload[5] = (uint8_t)(x2 >> 8);
    payload[6] = (uint8_t)(y2 & 0xFFu);
    payload[7] = (uint8_t)(y2 >> 8);
    payload[8] = color & 0x0Fu;

    return MPU_Send(CMD_SET_LINE, payload, sizeof(payload));
}


bool MPU_SendSetRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t color)
{
    uint8_t payload[9];

    payload[0] = (uint8_t)(x1 & 0xFFu);
    payload[1] = (uint8_t)(x1 >> 8);
    payload[2] = (uint8_t)(y1 & 0xFFu);
    payload[3] = (uint8_t)(y1 >> 8);
    payload[4] = (uint8_t)(x2 & 0xFFu);
    payload[5] = (uint8_t)(x2 >> 8);
    payload[6] = (uint8_t)(y2 & 0xFFu);
    payload[7] = (uint8_t)(y2 >> 8);
    payload[8] = color;

    return MPU_Send(CMD_SET_RECT, payload, sizeof(payload));
}


bool MPU_SendSetRectEx(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t border_color, uint8_t fill_color)
{
    uint8_t color = MAKE_COLOR_BF(border_color, fill_color);
    return MPU_SendSetRect(x1, y1, x2, y2, color);
}


bool MPU_SendSetCircle(uint16_t x, uint16_t y, uint16_t radius, uint8_t color)
{
    uint8_t payload[7];

    payload[0] = (uint8_t)(x & 0xFFu);
    payload[1] = (uint8_t)(x >> 8);
    payload[2] = (uint8_t)(y & 0xFFu);
    payload[3] = (uint8_t)(y >> 8);
    payload[4] = (uint8_t)(radius & 0xFFu);
    payload[5] = (uint8_t)(radius >> 8);
    payload[6] = color;

    return MPU_Send(CMD_SET_CIRCLE, payload, sizeof(payload));
}


bool MPU_SendSetCircleEx(uint16_t x, uint16_t y, uint16_t radius, uint8_t border_color, uint8_t fill_color)
{
    uint8_t color = MAKE_COLOR_BF(border_color, fill_color);
    return MPU_SendSetCircle(x, y, radius, color);
}


bool MPU_SendSetString(uint16_t x, uint16_t y, uint8_t color, const char *text)
{
    uint8_t payload[LINK_MAX_PAYLOAD];

    payload[0] = (uint8_t)(x & 0xFFu);
    payload[1] = (uint8_t)(x >> 8);
    payload[2] = (uint8_t)(y & 0xFFu);
    payload[3] = (uint8_t)(y >> 8);
    payload[4] = color & 0x0Fu;

    uint8_t idx = 5;
    if (text != NULL)
    {
        while (*text != '\0' && idx < (LINK_MAX_PAYLOAD - 1))
        {
            payload[idx++] = (uint8_t)(*text++);
        }
    }
    payload[idx++] = 0;

    return MPU_Send(CMD_SET_STRING, payload, idx);
}