#ifndef __CHASSIS_SPEED_H
#define __CHASSIS_SPEED_H

#include "stm32f10x.h"

/*
 * 编码器增量 -> 轮速
 * 返回单位：mm/s
 *
 * 依赖 config.h:
 * WHEEL_DIAMETER, ENCODER_PPR, GEAR_RATIO
 *
 * 默认采用 4倍频计数：
 * ticks_per_wheel_rev = ENCODER_PPR * GEAR_RATIO * 4
 */
int32_t ChassisSpeed_DeltaToMmps(int16_t delta_ticks, uint16_t dt_ms);

#endif
