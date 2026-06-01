#ifndef __IWDG_H
#define __IWDG_H

#include "stm32f10x.h"

void IWDG_Init(uint16_t Prescaler, uint16_t Reload);  // 初始化独立看门狗
void IWDG_Feed(void);                                  // 喂狗

#endif