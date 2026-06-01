#ifndef __FLYT_SERIAL_H
#define __FLYT_SERIAL_H

#include "stm32f10x.h"
#include "FlytProtocol.h"

/*
 * USART1 专用于飞腾派通信。
 *
 * PA9  -> USART1_TX -> USB-TTL RX / 飞腾派串口 RX
 * PA10 -> USART1_RX <- USB-TTL TX / 飞腾派串口 TX
 * Baudrate: 115200, 8N1
 *
 * 注意：
 * 使用本模块时，不要再编译旧 Serial.c 中的 USART1_IRQHandler，
 * 否则会出现 USART1_IRQHandler 重复定义。
 */

void FlytSerial_Init(void);
void FlytSerial_SendBytes(const uint8_t *buf, uint16_t len);
void FlytSerial_SendUplink(int32_t left_total_ticks,
                           int32_t right_total_ticks,
                           float gyro_z_dps,
                           uint8_t status);

/* 有新命令返回 1，并复制到 out_cmd；没有则返回 0 */
uint8_t FlytSerial_GetLatestCmd(FlytCmd_t *out_cmd);

#endif
