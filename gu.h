#ifndef GU_H
#define GU_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

/*
 * GU (SPI Slave) Interface
 */

void GU_Init(void);
void GU_Service(void);
bool GU_GetCommand(uint8_t *type, uint8_t *data, uint8_t *len);

void GU_CS_ISR(void);
void GU_SPI_STC_ISR(void);

#endif