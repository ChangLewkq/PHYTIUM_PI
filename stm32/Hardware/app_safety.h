#ifndef __APP_SAFETY_H
#define __APP_SAFETY_H

#include "stm32f10x.h"
#include <stdint.h>

/* ========== STM32 本地超声波安全保护 ==========
 * 25cm~10cm：只限制“前进速度”，允许后退/转向脱困
 * <=10cm：禁止继续前进，但允许后退/原地转向脱困
 * 测距函数 Ultrasonic_Read() 为阻塞式，所以低频调用，默认 100ms 一次。
 */
#define APP_SAFETY_ULTRA_PERIOD_MS         100
#define APP_SAFETY_ULTRA_SLOW_DIST_M       0.25f
#define APP_SAFETY_ULTRA_STOP_DIST_M       0.10f
#define APP_SAFETY_ULTRA_VALID_MIN_M       0.02f
#define APP_SAFETY_ULTRA_VALID_MAX_M       2.00f
#define APP_SAFETY_ULTRA_LIMIT_MMPS        80

#define APP_SAFETY_RESULT_NONE             0x00
#define APP_SAFETY_RESULT_LIMIT            0x01
#define APP_SAFETY_RESULT_STOP             0x02

void AppSafety_Init(void);

void AppSafety_SetPiTimeout(uint8_t timeout);
uint8_t AppSafety_GetPiTimeout(void);

void AppSafety_SetUltrasonicEnable(uint8_t enable);
uint8_t AppSafety_GetUltrasonicEnable(void);

void AppSafety_Update(uint16_t dt_ms);
uint8_t AppSafety_ApplyUltrasonicToTargets(int32_t *left_mmps, int32_t *right_mmps);
uint8_t AppSafety_BuildStatusByte(uint8_t estop_active);

float AppSafety_GetUltraDistM(void);
uint8_t AppSafety_GetUltraValid(void);
uint8_t AppSafety_GetUltraLimited(void);
uint8_t AppSafety_GetUltraStop(void);

#endif
