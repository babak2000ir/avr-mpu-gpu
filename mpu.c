/*
 * mpu_link.c
 *
 * ATtiny84 / ATtiny84A
 *
 * MPU = SPI master
 * GU  = SPI slave
 *
 * Reliable queued SPI protocol.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <util/delay.h>

#include "protocol.h"


/* ============================================================
 * Pin configuration
 * ============================================================ */

/*
 * ATtiny84 USI pins:
 *
 * PA6 = DI  = MOSI
 * PA5 = DO  = MISO
 * PA4 = USCK
 */

#define SPI_MOSI_BIT     PA6
#define SPI_MISO_BIT     PA5
#define SPI_SCK_BIT      PA4

#define CS_PORT          PORTA
#define CS_DDR           DDRA
#define CS_BIT           PA7

#define READY_PIN        PB0


/* ============================================================
 * Configuration
 * ============================================================ */

/*
 * 0 = fastest USI master operation.
 *
 * At F_CPU = 8 MHz this is approximately:
 *
 *      SCK = F_CPU / 4 = 2 MHz
 *
 * IMPORTANT:
 *
 * The GU must be able to service USI overflow before the next
 * byte arrives. If your VGA ISR is long, lower this.
 *
 * See the explanation below for choosing this value.
 */
#define LINK_FAST_SPI    1


/* ============================================================
 * Time base
 * ============================================================ */

volatile uint32_t mpu_millis = 0;


/*
 * This Timer0 configuration is provided as a complete example.
 *
 * 8 MHz / 64 = 125 kHz
 * OCR0A = 124
 *
 * => 1 ms
 *
 * If your MPU already has a system tick, delete this timer
 * and use your existing millisecond counter instead.
 */
static void mpu_timer_init(void)
{
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS01) | _BV(CS00);

    OCR0A = 124;

    TIFR0 = _BV(OCF0A);
    TIMSK0 = _BV(OCIE0A);
}


ISR(TIM0_COMPA_vect)
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
    return (PINB & _BV(READY_PIN)) != 0;
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
 * USI SPI master
 * ============================================================ */

/*
 * The ATtiny84 has USI rather than the conventional SPCR/SPDR
 * SPI registers found on larger AVRs.
 *
 * Microchip documents the USI three-wire mode as SPI-compatible.
 */

/*
 * Fast transfer.
 *
 * The USI counter counts BOTH clock edges, so 16 edges
 * correspond to one byte.
 */
static uint8_t usi_master_transfer(uint8_t value)
{
    uint8_t a;
    uint8_t b;

    /*
     * Load transmit byte.
     */
    USIDR = value;

    /*
     * Clear overflow flag and reset the 4-bit counter.
     */
    USISR = _BV(USIOIF);

#if LINK_FAST_SPI

    /*
     * Software clock generation.
     *
     * This is the same basic mechanism documented by Microchip
     * for maximum-speed USI master operation.
     */
    a = _BV(USIWM0) |
        _BV(USITC);

    b = _BV(USIWM0) |
        _BV(USITC) |
        _BV(USICLK);

    USICR = a;
    USICR = b;

    USICR = a;
    USICR = b;

    USICR = a;
    USICR = b;

    USICR = a;
    USICR = b;

    USICR = a;
    USICR = b;

    USICR = a;
    USICR = b;

    USICR = a;
    USICR = b;

    USICR = a;
    USICR = b;

#else

    /*
     * Slower/polling version.
     *
     * Useful during initial testing if the GU VGA interrupt
     * is long.
     */
    a = _BV(USIWM0) |
        _BV(USICS0) |
        _BV(USITC);

    while (!(USISR & _BV(USIOIF)))
    {
        USICR = a;
    }

#endif

    return USIDR;
}


/* ============================================================
 * SPI initialization
 * ============================================================ */

static void mpu_spi_init(void)
{
    /*
     * MOSI and SCK outputs.
     * MISO input.
     */
    DDRA |= _BV(SPI_MOSI_BIT);
    DDRA |= _BV(SPI_SCK_BIT);

    DDRA &= ~_BV(SPI_MISO_BIT);

    /*
     * CS output, idle high.
     */
    CS_DDR |= _BV(CS_BIT);
    CS_PORT |= _BV(CS_BIT);

    /*
     * READY input.
     *
     * No internal pull-up.
     *
     * Use an external pull-down on the GU READY line.
     */
    DDRB &= ~_BV(READY_PIN);
    PORTB &= ~_BV(READY_PIN);

    /*
     * USI initially disabled.
     */
    USICR = _BV(USIWM0);

    USISR = _BV(USIOIF);
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
bool GU_Send(uint8_t type,
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
    MPU_LINK_WAIT_STATUS,
    MPU_LINK_RETRY_WAIT
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
     * the USI transaction state.
     */
    _delay_us(2);

    usi_master_transfer(LINK_SOF);

    usi_master_transfer(p->seq);
    usi_master_transfer(p->len);
    usi_master_transfer(p->type);

    for (uint8_t i = 0; i < p->len; i++)
        usi_master_transfer(p->data[i]);

    crc = link_packet_crc(p);

    usi_master_transfer((uint8_t)(crc & 0xFF));
    usi_master_transfer((uint8_t)(crc >> 8));

    cs_high();
}


/* ============================================================
 * Poll GU status
 * ============================================================ */

static uint8_t poll_status(uint8_t *sequence)
{
    uint8_t status;
    uint8_t seq;

    cs_low();

    _delay_us(2);

    /*
     * First byte tells GU this is a status transaction.
     */
    usi_master_transfer(LINK_STATUS_CMD);

    /*
     * GU responds with:
     *
     * byte 1 = status
     * byte 2 = sequence
     */
    status = usi_master_transfer(0x00);
    seq    = usi_master_transfer(0x00);

    cs_high();

    *sequence = seq;

    return status;
}


/* ============================================================
 * Communication service
 * ============================================================ */

/*
 * Call this VERY frequently from the MPU main loop.
 *
 * It never waits for READY.
 * It never waits for an ACK.
 *
 * The only bounded blocking operation is the actual SPI
 * byte transfer, which at 2 MHz is only 4 us/byte.
 */
void GU_Service(void)
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

            link_state = MPU_LINK_WAIT_STATUS;

            return;


        case MPU_LINK_WAIT_STATUS:

            /*
             * GU uses READY to tell us that its status response
             * is ready.
             */
            if (gu_ready())
            {
                uint8_t status_seq;
                uint8_t status;

                status = poll_status(&status_seq);

                p = queue_head();

                if (p == NULL)
                {
                    link_state = MPU_LINK_IDLE;
                    return;
                }

                /*
                 * Never accept an ACK for the wrong packet.
                 */
                if (status_seq != p->seq)
                {
                    /*
                     * Something is wrong with synchronization.
                     *
                     * Do not remove the packet.
                     */
                    if (retry_count < LINK_MAX_RETRIES)
                    {
                        retry_count++;
                        link_state = MPU_LINK_RETRY_WAIT;
                    }

                    return;
                }

                switch (status)
                {
                    case LINK_STATUS_ACK:

                        /*
                         * Packet is accepted by GU.
                         *
                         * NOW, and only now, remove it.
                         */
                        queue_pop();

                        retry_count = 0;

                        link_state = MPU_LINK_IDLE;

                        return;


                    case LINK_STATUS_NACK_CRC:

                    case LINK_STATUS_NACK_FORMAT:

                        /*
                         * Same packet must be transmitted again.
                         */
                        if (retry_count < LINK_MAX_RETRIES)
                        {
                            retry_count++;

                            link_state = MPU_LINK_RETRY_WAIT;
                        }

                        return;


                    case LINK_STATUS_NACK_BUSY:

                        /*
                         * GU couldn't accept it yet.
                         *
                         * Don't resend unnecessarily.
                         *
                         * Wait for another READY.
                         */
                        status_deadline =
                            now + LINK_ACK_TIMEOUT_MS;

                        return;


                    case LINK_STATUS_NACK_SEQ:

                    default:

                        /*
                         * Protocol synchronization error.
                         */
                        if (retry_count < LINK_MAX_RETRIES)
                        {
                            retry_count++;

                            link_state = MPU_LINK_RETRY_WAIT;
                        }

                        return;
                }
            }

            /*
             * GU hasn't produced a status yet.
             *
             * Do NOT block.
             */
            if ((int32_t)(now - status_deadline) >= 0)
            {
                /*
                 * ACK may simply have been lost.
                 *
                 * We deliberately DO NOT remove the packet.
                 */
                if (retry_count < LINK_MAX_RETRIES)
                {
                    retry_count++;
                    link_state = MPU_LINK_RETRY_WAIT;
                }
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

            status_deadline =
                now + LINK_ACK_TIMEOUT_MS;

            link_state = MPU_LINK_WAIT_STATUS;

            return;
    }
}


/* ============================================================
 * Optional status helpers
 * ============================================================ */

bool GU_IsBusy(void)
{
    return link_state != MPU_LINK_IDLE;
}


uint8_t GU_QueuedPackets(void)
{
    return tx_count;
}


uint8_t GU_RetryCount(void)
{
    return retry_count;
}


/* ============================================================
 * MPU initialization
 * ============================================================ */

void GU_Link_Init(void)
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