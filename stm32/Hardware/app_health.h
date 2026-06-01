#ifndef __APP_HEALTH_H
#define __APP_HEALTH_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * AppHealth
 *
 * RT-Thread 任务心跳监测模块。
 *
 * V1.2 新增：
 * - 关键线程健康判断
 * - 看门狗喂狗许可判断
 *
 * 作用：
 * - 记录各关键线程循环次数
 * - 记录各关键线程最近一次心跳 tick
 * - 蓝牙 [TASK] 可打印任务状态
 * - app_status 可基于健康状态决定是否喂 IWDG
 *
 * 注意：
 * - 本模块只判断“线程是否还在运行”。
 * - 不判断 PID 效果、编码器方向、传感器精度。
 */

typedef enum
{
    APP_HEALTH_INIT = 0,
    APP_HEALTH_MOTOR,
    APP_HEALTH_PI,
    APP_HEALTH_DEBUG,
    APP_HEALTH_STATUS,
    APP_HEALTH_MAX
} AppHealthId_t;

void AppHealth_Init(void);
void AppHealth_Beat(AppHealthId_t id);

uint32_t AppHealth_GetCount(AppHealthId_t id);
uint32_t AppHealth_GetLastTick(AppHealthId_t id);
uint32_t AppHealth_GetAgeMs(AppHealthId_t id);

/*
 * 检查单个任务是否在 max_age_ms 内更新过心跳。
 * 返回 1：正常
 * 返回 0：超时或从未更新
 */
uint8_t AppHealth_IsAlive(AppHealthId_t id, uint32_t max_age_ms);

/*
 * 检查关键线程是否都正常。
 *
 * 默认判定：
 * - motor  必须活着，<= 200ms
 * - pi     必须活着，<= 200ms
 * - debug  必须活着，<= 500ms
 * - status 必须活着，<= 500ms
 *
 * init 线程低频心跳，不作为喂狗必要条件。
 */
uint8_t AppHealth_AllCriticalAlive(void);

void AppHealth_Print(void);

#endif
