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

    /* Example graphics command transmission */
    uint8_t demo_cmd[] = { 0x10, 0x20, 0x30, 0x40 };
    MPU_Send(0x01, demo_cmd, sizeof(demo_cmd));

    for (;;)
    {
        /* Service transmit queue and protocol state machine */
        MPU_Service();

        /* Main MPU application tasks */
    }
}