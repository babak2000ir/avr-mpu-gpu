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

/*
 * Graphics Command Queueing Helpers
 */
bool MPU_SendSetPixel(uint16_t x, uint16_t y, uint8_t color);
bool MPU_SendSetLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t color);
bool MPU_SendSetRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t color);
bool MPU_SendSetRectEx(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t border_color, uint8_t fill_color);
bool MPU_SendSetCircle(uint16_t x, uint16_t y, uint16_t radius, uint8_t color);
bool MPU_SendSetCircleEx(uint16_t x, uint16_t y, uint16_t radius, uint8_t border_color, uint8_t fill_color);
bool MPU_SendSetString(uint16_t x, uint16_t y, uint8_t color, const char *text);

/* Command function aliases */
#define MPU_CmdSetPixel    MPU_SendSetPixel
#define MPU_SetPixel       MPU_SendSetPixel
#define MPU_CmdSetLine     MPU_SendSetLine
#define MPU_SetLine        MPU_SendSetLine
#define MPU_CmdSetRect     MPU_SendSetRect
#define MPU_SetRect        MPU_SendSetRect
#define MPU_CmdSetCircle   MPU_SendSetCircle
#define MPU_SetCircle      MPU_SendSetCircle
#define MPU_CmdSetString   MPU_SendSetString
#define MPU_SetString      MPU_SendSetString

/* Backwards compatibility aliases */
#define GU_Link_Init  MPU_Init
#define GU_Service    MPU_Service
#define GU_Send       MPU_Send

#endif
