#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"

/* Functions from gu_link.c */
void GU_Link_Init(void);
void GU_Link_Service(void);
bool GU_GetCommand(uint8_t *type,
                   uint8_t *data,
                   uint8_t *len);


/*
 * Your VGA timer ISR.
 *
 * THIS IS THE HIGHEST PRIORITY JOB.
 *
 * Keep it exactly as timing-critical as necessary.
 */
ISR(TIM1_COMPA_vect)
{
    /*
     * VGA bit banging here.
     *
     * Do not call:
     *
     *   GU_Link_Service()
     *   GU_GetCommand()
     *
     * from here.
     */
}


static void process_gu_command(uint8_t type,
                               uint8_t *data,
                               uint8_t len)
{
    switch (type)
    {
        case 0x01:
            /*
             * Example:
             * draw something
             */
            break;

        case 0x02:
            /*
             * Example:
             * update a display register
             */
            break;

        default:
            /*
             * Unknown command.
             *
             * This should normally never happen because
             * the MPU and GU share the protocol definition.
             */
            break;
    }
}


int main(void)
{
    uint8_t type;
    uint8_t len;
    uint8_t data[LINK_MAX_PAYLOAD];

    /*
     * Initialize VGA hardware/timer FIRST if VGA timing
     * is your primary concern.
     */
    /*
     * VGA_Init();
     */

    GU_Link_Init();

    /*
     * Once VGA and other critical initialization is complete,
     * GU_Link_Service() will eventually raise READY.
     */

    for (;;)
    {
        /*
         * Communication protocol.
         *
         * Very short.
         */
        GU_Link_Service();


        /*
         * Process commands outside interrupts.
         */
        while (GU_GetCommand(&type, data, &len))
        {
            process_gu_command(type, data, len);

            /*
             * You can limit this to one command per main-loop
             * iteration if command execution can become lengthy.
             */
        }


        /*
         * Other GU background work.
         */
    }
}