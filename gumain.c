#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"
#include "gu.h"

/*
 * Interrupt Service Routines
 */

/*
 * VGA timer ISR - highest priority task
 */
ISR(TIM1_COMPA_vect)
{
    /*
     * VGA timing-critical code
     */
}

/*
 * CS Pin Change ISR
 */
ISR(PCINT0_vect)
{
    GU_CS_ISR();
}

/*
 * USI Overflow ISR
 */
ISR(USI_OVF_vect)
{
    GU_USI_OVF_ISR();
}

static void process_gu_command(uint8_t type,
                               uint8_t *data,
                               uint8_t len)
{
    (void)data;
    (void)len;

    switch (type)
    {
        case 0x01:
            /*
             * Example: draw command
             */
            break;

        case 0x02:
            /*
             * Example: update display register
             */
            break;

        default:
            break;
    }
}

int main(void)
{
    uint8_t type;
    uint8_t len;
    uint8_t data[LINK_MAX_PAYLOAD];

    /* Initialize GU hardware and protocol handler */
    GU_Init();

    for (;;)
    {
        /* Process incoming SPI protocol transactions */
        GU_Service();

        /* Process commands outside interrupts */
        while (GU_GetCommand(&type, data, &len))
        {
            process_gu_command(type, data, len);
        }
    }
}