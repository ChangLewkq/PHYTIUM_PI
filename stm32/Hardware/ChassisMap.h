#ifndef __CHASSIS_MAP_H
#define __CHASSIS_MAP_H

#include "stm32f10x.h"

/*
 * 物理定义：
 * - left/right：以小车自身朝向为准
 * - wheel PWM > 0：该轮正转，推动车体前进
 * - encoder delta > 0：该轮正转，推动车体前进
 */

void ChassisMap_SetWheelPWM(int16_t left_pwm, int16_t right_pwm);
void ChassisMap_Stop(void);

int16_t ChassisMap_GetLeftDelta(void);
int16_t ChassisMap_GetRightDelta(void);

#endif
