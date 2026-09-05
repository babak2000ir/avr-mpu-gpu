/*
 * mpumain.c
 * ATtiny84 / ATtiny84A
 * MPU = SPI master
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"
#include "mpu.h"

/*
 * Timer0 Interrupt Service Routine (1ms tick)
 */
ISR(TIM0_COMPA_vect)
{
    MPU_TimerTick();
}

int main(void)
{
    /* Initialize MPU SPI master hardware and queues */
    MPU_Init();

    /* Queue graphics commands */
    MPU_SendSetPixel(10, 20, 0x0F);
    MPU_SendSetLine(0, 0, 100, 100, 0x05);
    MPU_SendSetRectEx(10, 10, 50, 50, 0x01, 0x02);
    MPU_SendSetCircleEx(30, 30, 15, 0x03, 0x04);
    MPU_SendSetString(5, 5, 0x0E, "Hello GU!");

    for (;;)
    {
        /* Service transmit queue and protocol state machine */
        MPU_Service();

        /* Main MPU application tasks */
    }
}