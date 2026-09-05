#ifndef MPU_H
#define MPU_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

/*
 * MPU (SPI Master) Interface
 */

void MPU_Init(void);
void MPU_Service(void);
bool MPU_Send(uint8_t type, const uint8_t *data, uint8_t len);
void MPU_TimerTick(void);

bool MPU_IsBusy(void);
uint8_t MPU_QueuedPackets(void);
uint8_t MPU_RetryCount(void);

/* Backwards compatibility aliases */
#define GU_Link_Init  MPU_Init
#define GU_Service    MPU_Service
#define GU_Send       MPU_Send

#endif
