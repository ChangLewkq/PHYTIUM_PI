#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void Encoder_Init(void);
int16_t Encoder_Get(uint8_t n);         // 获取增量值（原函数，保留）
int32_t Encoder_GetTotal(uint8_t n);    // 获取累计值（新增）

#endif