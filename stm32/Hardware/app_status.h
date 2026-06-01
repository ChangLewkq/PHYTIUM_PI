#ifndef __APP_STATUS_H
#define __APP_STATUS_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * AppStatus V1.4
 *
 * 功能：
 * 1. PC13 LED 状态灯
 * 2. OLED 低频状态显示
 * 3. IWDG 硬件看门狗
 *
 * V1.4 重点：
 * - 正式启用 IWDG
 * - 采用“延迟启动”策略
 * - 只有关键线程心跳全部正常一段时间后，才真正启动 IWDG
 * - IWDG 启动后，只有关键线程继续正常才喂狗
 *
 * 这样可以避免上电初始化、MPU 校准期间误复位。
 */

#define APP_STATUS_USE_OLED          1
#define APP_STATUS_USE_IWDG          1

#define APP_STATUS_LED_NORMAL_MS     800
#define APP_STATUS_LED_MOVE_MS       250
#define APP_STATUS_LED_DEBUG_MS      120
#define APP_STATUS_LED_TIMEOUT_MS    120

#define APP_STATUS_OLED_PERIOD_MS    500
#define APP_STATUS_WDG_PERIOD_MS     100

/*
 * IWDG 延迟启动条件：
 * 关键线程全部健康，并持续累计达到这个时间后，才启动 IWDG。
 */
#define APP_STATUS_IWDG_START_DELAY_MS  1500

void AppStatus_Init(void);
void AppStatus_Step(uint16_t dt_ms);

/*
 * 1：关键任务健康，可以喂狗
 * 0：关键任务异常，不应喂狗
 */
uint8_t AppStatus_IsWatchdogFeedAllowed(void);

/*
 * 1：IWDG 已经真正启动
 * 0：IWDG 尚未启动，仍在等待系统稳定
 */
uint8_t AppStatus_IsWatchdogStarted(void);

#endif
