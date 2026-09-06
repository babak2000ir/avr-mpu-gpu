/*
 * gu_link.c
 *
 * ATmega328P
 *
 * GU = SPI slave
 *
 * VGA timing always has priority.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"
#include "gu.h"


/* ============================================================
 * Pins
 * ============================================================ */

#define SPI_MOSI_BIT     PB3
#define SPI_MISO_BIT     PB4
#define SPI_SCK_BIT      PB5
#define SPI_SS_BIT       PB2

#define READY_BIT        PD2


/* ============================================================
 * Packet RX buffer
 * ============================================================ */

static volatile uint8_t rx_buffer[LINK_MAX_PAYLOAD + 4];

/*
 * Layout:
 *
 * rx_buffer[0] = SEQ
 * rx_buffer[1] = LEN
 * rx_buffer[2] = TYPE
 * rx_buffer[3...] = PAYLOAD
 */

static volatile uint8_t rx_index = 0;

static volatile uint8_t rx_expected_length = 0;

static volatile bool rx_transaction_active = false;

static volatile bool rx_transaction_complete = false;

static volatile bool rx_is_status_transaction = false;

static volatile bool rx_protocol_error = false;


/* ============================================================
 * Status response
 * ============================================================ */

static volatile uint8_t status_to_send = LINK_STATUS_NONE;
static volatile uint8_t status_sequence = 0;


/*
 * Status response transaction has completed.
 */
static volatile bool status_transaction_complete = false;


/* ============================================================
 * GU command queue
 * ============================================================ */

static GuCommand command_queue[GU_INSTRUCTION_QUEUE_SIZE];

static uint8_t command_head = 0;
static uint8_t command_tail = 0;
static uint8_t command_count = 0;


/* ============================================================
 * Sequence tracking
 * ============================================================ */

static uint8_t expected_sequence = 0;


/*
 * We have accepted at least one packet.
 *
 * Used to recognize retransmission of the last packet.
 */
static bool have_last_sequence = false;

static uint8_t last_accepted_sequence = 0;


/* ============================================================
 * READY control
 * ============================================================ */

static inline void gu_ready_high(void)
{
    PORTD |= _BV(READY_BIT);
}


static inline void gu_ready_low(void)
{
    PORTD &= ~_BV(READY_BIT);
}

/* ============================================================
 * SPI helpers
 * ============================================================ */

static inline void spi_enable(void)
{
    /* SPI slave, mode 0, MSB first. */
    SPCR = _BV(SPE) | _BV(SPIE);
}


/* ============================================================
 * CS pin-change interrupt
 * ============================================================ */

/*
 * PB2 belongs to PCINT2, therefore:
 *
 *     PCMSK0 bit 2
 *     PCIE0
 *     PCINT0_vect
 *
 * are used.
 */

void GU_CS_ISR(void)
{
     uint8_t pins = PINB;

     if (!(pins & _BV(SPI_SS_BIT)))
    {
        /*
         * CS falling.
         *
         * Start new SPI transaction.
         */

        rx_transaction_active = true;

        rx_transaction_complete = false;

        status_transaction_complete = false;

        rx_index = 0;

        rx_expected_length = 0;

        rx_protocol_error = false;

        rx_is_status_transaction = false;

        /*
         * GU cannot accept another DATA packet until this
         * transaction has been completely handled.
         */
        gu_ready_low();

        /*
         * First byte received determines transaction type.
         */
        SPDR = 0x00;

    }
    else
    {
        /*
         * CS rising.
         *
         * End of transaction.
         */

        rx_transaction_active = false;

        /*
         * MISO can be driven low while idle.
         */
        PORTB &= ~_BV(SPI_MISO_BIT);

        /*
         * Tell main loop that something needs processing.
         */
        if (rx_is_status_transaction)
            status_transaction_complete = true;
        else
            rx_transaction_complete = true;
    }
}


/* ============================================================
 * SPI transfer complete ISR
 * ============================================================ */

/*
 * VERY IMPORTANT:
 *
 * Do not:
 *
 *   - calculate CRC
 *   - execute commands
 *   - manipulate VGA state
 *   - wait for anything
 *
 * here.
 *
 * This ISR should be as short as possible.
 */

void GU_SPI_STC_ISR(void)
{
    uint8_t value;

    /*
     * Read received byte immediately.
     *
    * The SPI data register has no receive FIFO.
     */
    value = SPDR;


    /* --------------------------------------------------------
     * First byte of transaction
     * -------------------------------------------------------- */

    if (rx_index == 0)
    {
        if (value == LINK_STATUS_CMD)
        {
            /*
             * Status request.
             */
            rx_is_status_transaction = true;

            /*
             * Next received byte gets status.
             */
            SPDR = status_to_send;
        }
        else if (value == LINK_SOF)
        {
            /*
             * DATA transaction.
             */
            rx_is_status_transaction = false;

            /*
             * Next byte will be SEQ.
             */
            SPDR = 0x00;
        }
        else
        {
            /*
             * Unknown transaction.
             */
            rx_protocol_error = true;

            SPDR = 0xFF;
        }

        rx_index = 1;

        return;
    }


    /* --------------------------------------------------------
     * STATUS transaction
     * -------------------------------------------------------- */

    if (rx_is_status_transaction)
    {
        /*
         * rx_index == 1:
         *
         * The master has just clocked STATUS.
         *
         * Next byte should return sequence.
         */
        if (rx_index == 1)
        {
            SPDR = status_sequence;
        }
        else
        {
            /*
             * Extra bytes get zero.
             */
            SPDR = 0x00;
        }

        rx_index++;

        return;
    }


    /* --------------------------------------------------------
     * DATA transaction
     * -------------------------------------------------------- */

    if (rx_index == 1)
    {
        /*
         * SEQ
         */
        rx_buffer[0] = value;

        SPDR = 0x00;

        rx_index++;

        return;
    }


    if (rx_index == 2)
    {
        /*
         * LEN
         */
        rx_buffer[1] = value;

        /*
         * Reject impossible length immediately.
         */
        if (value > LINK_MAX_PAYLOAD)
        {
            rx_protocol_error = true;
            rx_expected_length = 0;
        }
        else
        {
            rx_expected_length = value;
        }

        SPDR = 0x00;

        rx_index++;

        return;
    }


    if (rx_index == 3)
    {
        /*
         * TYPE
         */
        rx_buffer[2] = value;

        SPDR = 0x00;

        rx_index++;

        return;
    }


    /*
     * Payload starts at rx_buffer[3].
     *
     * We don't parse it here.
     */
    if (!rx_protocol_error)
    {
        uint8_t payload_index = rx_index - 4;

        /*
         * Receive payload.
         */
        if (payload_index < rx_expected_length)
        {
            rx_buffer[3 + payload_index] = value;

            SPDR = 0x00;

            rx_index++;

            return;
        }
    }


    /*
     * CRC bytes.
     *
     * We store them immediately after payload.
     */
    if (!rx_protocol_error)
    {
        uint8_t crc_index = rx_index - 4 - rx_expected_length;

        if (crc_index == 0)
        {
            /*
             * CRC low byte.
             */
            rx_buffer[3 + rx_expected_length] = value;

            SPDR = 0x00;

            rx_index++;

            return;
        }

        if (crc_index == 1)
        {
            /*
             * CRC high byte.
             */
            rx_buffer[4 + rx_expected_length] = value;

            /*
             * Packet reception is finished.
             *
             * Main loop will validate it.
             */
            rx_transaction_complete = true;

            /*
             * We don't need another useful byte.
             */
            SPDR = 0x00;

            rx_index++;

            return;
        }
    }


    /*
     * Anything beyond the expected packet is ignored.
     */
    SPDR = 0x00;

    rx_index++;
}


/* ============================================================
 * GU SPI initialization
 * ============================================================ */

static void gu_spi_init(void)
{
    /*
    * MOSI and SCK inputs.
     */
    DDRB &= ~(_BV(SPI_MOSI_BIT) | _BV(SPI_SCK_BIT));

    /*
     * DO/MISO output.
     */
    DDRB |= _BV(SPI_MISO_BIT);

    PORTB &= ~_BV(SPI_MISO_BIT);

    /*
     * CS input with pull-up.
     *
     * MPU drives it actively.
     */
    DDRB &= ~_BV(SPI_SS_BIT);
    PORTB |= _BV(SPI_SS_BIT);

    /*
     * READY output.
     */
    DDRD |= _BV(READY_BIT);

    gu_ready_low();


    /*
    * Enable PB2 pin-change interrupt.
     */
    PCMSK0 |= _BV(PCINT2);

    /*
     * Clear pending pin-change interrupt.
     */
    PCIFR = _BV(PCIF0);

    /*
    * Enable port B pin-change interrupts.
     */
    PCICR |= _BV(PCIE0);

    /*
     * Keep SPI enabled so the slave is ready before the first clock edge
     * following SS assertion. Hardware gates MISO while SS is high.
     */
    spi_enable();
}


/* ============================================================
 * Command queue
 * ============================================================ */

static bool command_queue_push(const LinkPacket *packet)
{
    if (command_count >= GU_INSTRUCTION_QUEUE_SIZE)
        return false;

    GuCommand *cmd = &command_queue[command_tail];

    cmd->seq = packet->seq;
    cmd->type = packet->type;
    cmd->len = packet->len;

    for (uint8_t i = 0; i < packet->len; i++)
        cmd->data[i] = packet->data[i];

    command_tail++;

    if (command_tail >= GU_INSTRUCTION_QUEUE_SIZE)
        command_tail = 0;

    command_count++;

    return true;
}


static bool command_queue_pop(GuCommand *cmd)
{
    if (command_count == 0)
        return false;

    *cmd = command_queue[command_head];

    command_head++;

    if (command_head >= GU_INSTRUCTION_QUEUE_SIZE)
        command_head = 0;

    command_count--;

    return true;
}


/* ============================================================
 * Convert received buffer to LinkPacket
 * ============================================================ */

static void make_received_packet(LinkPacket *packet)
{
    packet->seq  = rx_buffer[0];
    packet->len  = rx_buffer[1];
    packet->type = rx_buffer[2];

    for (uint8_t i = 0; i < packet->len; i++)
        packet->data[i] = rx_buffer[3 + i];
}


static uint16_t received_crc(void)
{
    uint8_t index = 3 + rx_buffer[1];

    uint16_t crc;

    crc  = rx_buffer[index];
    crc |= (uint16_t)rx_buffer[index + 1] << 8;

    return crc;
}


/* ============================================================
 * Process completed DATA transaction
 * ============================================================ */

static void process_received_data(void)
{
    LinkPacket packet;

    uint16_t calculated_crc;
    uint16_t received;

    /*
     * Take a local copy.
     *
     * This means the rest of the system never works directly
     * on the volatile SPI buffer.
     */
    if (rx_protocol_error)
    {
        status_to_send = LINK_STATUS_NACK_FORMAT;
        status_sequence = 0;

        gu_ready_high();

        return;
    }

    /*
     * Sanity check.
     */
    if (rx_buffer[1] > LINK_MAX_PAYLOAD)
    {
        status_to_send = LINK_STATUS_NACK_FORMAT;
        status_sequence = rx_buffer[0];

        gu_ready_high();

        return;
    }

    make_received_packet(&packet);

    status_sequence = packet.seq;

    calculated_crc = link_packet_crc(&packet);
    received = received_crc();

    /*
     * CRC check comes BEFORE sequence processing.
     */
    if (calculated_crc != received)
    {
        status_to_send = LINK_STATUS_NACK_CRC;

        /*
         * Do NOT modify expected_sequence.
         */

        gu_ready_high();

        return;
    }


    /* --------------------------------------------------------
     * Duplicate detection
     * -------------------------------------------------------- */

    if (have_last_sequence &&
        packet.seq == last_accepted_sequence)
    {
        /*
         * This is a retransmission of the last successfully
         * accepted packet.
         *
         * DO NOT put it in the command queue.
         *
         * DO NOT execute it again.
         *
         * Just ACK it again.
         */
        status_to_send = LINK_STATUS_ACK;

        gu_ready_high();

        return;
    }


    /* --------------------------------------------------------
     * Sequence check
     * -------------------------------------------------------- */

    if (packet.seq != expected_sequence)
    {
        status_to_send = LINK_STATUS_NACK_SEQ;

        gu_ready_high();

        return;
    }


    /* --------------------------------------------------------
     * Queue space check
     * -------------------------------------------------------- */

    if (!command_queue_push(&packet))
    {
        /*
         * Important:
         *
         * We do NOT acknowledge a packet that we haven't
         * stored.
         */
        status_to_send = LINK_STATUS_NACK_BUSY;

        gu_ready_high();

        return;
    }


    /* --------------------------------------------------------
     * Packet accepted
     * -------------------------------------------------------- */

    last_accepted_sequence = packet.seq;

    have_last_sequence = true;

    expected_sequence++;

    /*
     * ACK means:
     *
     * "The GU has safely accepted this command into its
     *  command queue."
     *
     * It does NOT mean the command has already executed.
     */
    status_to_send = LINK_STATUS_ACK;

    /*
     * Status is now available.
     */
    gu_ready_high();
}


/* ============================================================
 * Process STATUS transaction completion
 * ============================================================ */

static void process_status_transaction(void)
{
    /*
     * STATUS has now been clocked out.
     *
     * Decide what READY means for the next transaction.
     */

    status_transaction_complete = false;

    /*
     * If the previous result was NACK_BUSY, keep the same
     * status available. The MPU will poll again.
     */
    if (status_to_send == LINK_STATUS_NACK_BUSY)
    {
        if (command_count < GU_INSTRUCTION_QUEUE_SIZE)
        {
            /*
             * Queue space is now available.
             *
             * But the packet has NOT been accepted yet.
             *
             * We want the MPU to retransmit DATA.
             */
            status_to_send = LINK_STATUS_NACK_BUSY;

            gu_ready_high();
        }
        else
        {
            gu_ready_low();
        }

        return;
    }


    /*
     * For ACK:
     *
     * the MPU can send the next packet.
     */
    if (status_to_send == LINK_STATUS_ACK)
    {
        if (command_count < GU_INSTRUCTION_QUEUE_SIZE)
            gu_ready_high();
        else
            gu_ready_low();

        /*
         * Status can remain ACK because a DATA transaction
         * will overwrite the next response.
         */
        return;
    }


    /*
     * CRC/format/sequence failure:
     *
     * READY means:
     *
     * "Send the DATA transaction again."
     */
    if (status_to_send == LINK_STATUS_NACK_CRC ||
        status_to_send == LINK_STATUS_NACK_FORMAT ||
        status_to_send == LINK_STATUS_NACK_SEQ)
    {
        gu_ready_high();

        return;
    }
}


/* ============================================================
 * GU communication service
 * ============================================================ */

/*
 * Call this from the GU main loop.
 *
 * NEVER call it from the VGA ISR.
 */
void GU_Service(void)
{
    /*
     * DATA transaction finished.
     *
     * Validate it here.
     */
    if (rx_transaction_complete)
    {
        rx_transaction_complete = false;

        /*
         * Keep READY low while processing.
         */
        gu_ready_low();

        process_received_data();

        return;
    }


    /*
     * STATUS transaction finished.
     */
    if (status_transaction_complete)
    {
        process_status_transaction();

        return;
    }
}


/* ============================================================
 * Retrieve commands for the GU application
 * ============================================================ */

/*
 * This is intentionally separate from GU_Link_Service().
 *
 * Your application calls this from the normal GU main loop.
 *
 * It NEVER runs inside the VGA timer ISR.
 */
bool GU_GetCommand(uint8_t *type,
                   uint8_t *data,
                   uint8_t *len)
{
    GuCommand cmd;

    if (!command_queue_pop(&cmd))
        return false;

    *type = cmd.type;
    *len = cmd.len;

    for (uint8_t i = 0; i < cmd.len; i++)
        data[i] = cmd.data[i];

    return true;
}


/* ============================================================
 * Initialization
 * ============================================================ */

void GU_Init(void)
{
    command_head = 0;
    command_tail = 0;
    command_count = 0;

    expected_sequence = 0;

    have_last_sequence = false;
    last_accepted_sequence = 0;

    status_to_send = LINK_STATUS_NONE;
    status_sequence = 0;

    rx_index = 0;
    rx_expected_length = 0;

    rx_transaction_active = false;
    rx_transaction_complete = false;
    status_transaction_complete = false;
    rx_is_status_transaction = false;
    rx_protocol_error = false;

    gu_spi_init();

    /*
     * Enable interrupts.
     *
    * TIMER1_COMPA_vect has higher interrupt priority than
    * SPI_STC_vect on the ATmega328P.
     */
    sei();

    /*
     * GU starts unavailable until its application has finished
     * initialization.
     */
    gu_ready_low();
}


/* ============================================================
 * Optional diagnostics
 * ============================================================ */

uint8_t GU_CommandQueueCount(void)
{
    return command_count;
}


uint8_t GU_ExpectedSequence(void)
{
    return expected_sequence;
}