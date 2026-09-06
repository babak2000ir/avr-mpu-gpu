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
ISR(TIMER1_COMPA_vect)
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
 * SPI transfer complete ISR
 */
ISR(SPI_STC_vect)
{
    GU_SPI_STC_ISR();
}

static void process_gu_command(uint8_t type,
                               uint8_t *data,
                               uint8_t len)
{
    switch (type)
    {
        case CMD_SET_PIXEL:
        {
            CmdSetPixel pixel;
            if (GU_ParseSetPixel(data, len, &pixel))
            {
                /* Handle setpixel: pixel.x, pixel.y, pixel.color */
            }
            break;
        }

        case CMD_SET_LINE:
        {
            CmdSetLine line;
            if (GU_ParseSetLine(data, len, &line))
            {
                /* Handle setline: line.x1, line.y1, line.x2, line.y2, line.color */
            }
            break;
        }

        case CMD_SET_RECT:
        {
            CmdSetRect rect;
            if (GU_ParseSetRect(data, len, &rect))
            {
                uint8_t border = COLOR_GET_BORDER(rect.color);
                uint8_t fill   = COLOR_GET_FILL(rect.color);
                /* Handle setrect: rect.x1, rect.y1, rect.x2, rect.y2, border, fill */
                (void)border;
                (void)fill;
            }
            break;
        }

        case CMD_SET_CIRCLE:
        {
            CmdSetCircle circle;
            if (GU_ParseSetCircle(data, len, &circle))
            {
                uint8_t border = COLOR_GET_BORDER(circle.color);
                uint8_t fill   = COLOR_GET_FILL(circle.color);
                /* Handle setcircle: circle.x, circle.y, circle.radius, border, fill */
                (void)border;
                (void)fill;
            }
            break;
        }

        case CMD_SET_STRING:
        {
            CmdSetString str;
            if (GU_ParseSetString(data, len, &str))
            {
                /* Handle setstring: str.x, str.y, str.color, str.text */
            }
            break;
        }

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