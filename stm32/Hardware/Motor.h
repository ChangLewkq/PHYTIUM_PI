#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_Init(void);
void Motor_SetPWM(uint8_t n, int8_t PWM);          // 原函数：-100~100
void Motor_SetSpeed(uint8_t n, float Speed);        // 新增：速度(m/s)控制
void Motor_Stop(void);                              // 新增：急停

#endif