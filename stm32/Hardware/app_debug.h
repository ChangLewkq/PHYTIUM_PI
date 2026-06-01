#ifndef __APP_DEBUG_H
#define __APP_DEBUG_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * AppDebug
 *
 * 功能：
 * 1. 管理蓝牙调试命令
 * 2. 管理蓝牙急停状态
 * 3. 管理蓝牙 DBG 周期状态输出
 * 4. 打印帮助与系统状态
 *
 * 说明：
 * - 当前仍使用原 BlueSerial.c 的命令输入格式：
 *   [STAT] / [F180] 或 STAT + CRLF / F180 + CRLF
 * - 本模块只负责“调试入口”和“参数调整”，不负责底层实时控制。
 * - 后续迁移 RT-Thread 时，本模块可变成 bluetooth_debug_task / shell_task。
 */

void AppDebug_Init(uint16_t ctrl_period_ms,
                   uint16_t report_period_ms,
                   uint16_t cmd_watchdog_ms);

void AppDebug_PrintHelp(void);
void AppDebug_PrintStatus(void);
void AppDebug_PrintStartup(void);

/*
 * 如果收到蓝牙运动命令且成功设置底盘目标，返回 1。
 * main.c 收到 1 后刷新命令看门狗。
 */
uint8_t AppDebug_ProcessRx(void);

void AppDebug_ReportStep(void);

uint8_t AppDebug_GetEstop(void);
uint8_t AppDebug_GetDebugEnable(void);

#endif
