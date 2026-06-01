#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

#include "stm32f10x.h"
#include <stdint.h>

#define BLUESERIAL_RX_PACKET_SIZE    100

extern char BlueSerial_RxPacket[BLUESERIAL_RX_PACKET_SIZE];
extern uint8_t BlueSerial_RxFlag;

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendString(char *String);
void BlueSerial_Printf(char *format, ...);

#endif
