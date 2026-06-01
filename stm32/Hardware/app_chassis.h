#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * AppChassis
 *
 * 功能：
 * 1. 管理左右轮 PID 参数
 * 2. 管理基础目标速度 base_l/base_r
 * 3. 叠加 AppImu 航向保持修正，生成 ctrl_l/ctrl_r
 * 4. 执行一轮底盘控制：编码器 -> 速度 -> PID -> PWM -> 上行
 * 5. 保存调试状态，供蓝牙 [STAT]/[DBG1] 输出
 *
 * 说明：
 * - 当前仍保持裸机 50ms 主循环调用。
 * - 后续迁移 RT-Thread 时，AppChassis_ControlAndUplinkStep() 可拆成 motor_ctrl_task + pi_comm_task。
 */

void AppChassis_Init(void);
void AppChassis_Stop(void);
void AppChassis_ClearDelta(void);
void AppChassis_ResetPid(void);

void AppChassis_UpdateTargetFromCmd(float linear_mps,
                                    float angular_radps,
                                    uint8_t estop_active);

/* 返回 1 表示成功设置；返回 0 表示 estop_active，已停车 */
uint8_t AppChassis_SetBluetoothWheelTarget(int32_t left_mmps,
                                           int32_t right_mmps,
                                           uint8_t estop_active);

void AppChassis_BuildControlTarget(void);
void AppChassis_ControlAndUplinkStep(uint8_t estop_active);

/* 参数设置 */
void AppChassis_SetKpLeft(float v);
void AppChassis_SetKpRight(float v);
void AppChassis_SetKpBoth(float v);

void AppChassis_SetKiLeft(float v);
void AppChassis_SetKiRight(float v);
void AppChassis_SetKiBoth(float v);

void AppChassis_SetFFLeft(float v);
void AppChassis_SetFFRight(float v);

void AppChassis_SetStartLeft(int16_t v);
void AppChassis_SetStartRight(int16_t v);

void AppChassis_SetMinControlSpeed(int32_t v);
void AppChassis_SetTargetLimit(int32_t v);
void AppChassis_SetPwmLimit(int16_t v);
void AppChassis_SetStep(int16_t v);

/* 参数读取 */
float AppChassis_GetKpLeft(void);
float AppChassis_GetKpRight(void);
float AppChassis_GetKiLeft(void);
float AppChassis_GetKiRight(void);
float AppChassis_GetFFLeft(void);
float AppChassis_GetFFRight(void);
int16_t AppChassis_GetStartLeft(void);
int16_t AppChassis_GetStartRight(void);
int32_t AppChassis_GetMinControlSpeed(void);
int32_t AppChassis_GetTargetLimit(void);
int16_t AppChassis_GetPwmLimit(void);
int16_t AppChassis_GetStep(void);

/* 状态读取 */
int32_t AppChassis_GetBaseLeftMmps(void);
int32_t AppChassis_GetBaseRightMmps(void);
int32_t AppChassis_GetCtrlLeftMmps(void);
int32_t AppChassis_GetCtrlRightMmps(void);

int16_t AppChassis_GetLastDeltaLeft(void);
int16_t AppChassis_GetLastDeltaRight(void);
int32_t AppChassis_GetLastActualLeftMmps(void);
int32_t AppChassis_GetLastActualRightMmps(void);
int16_t AppChassis_GetLastPwmLeft(void);
int16_t AppChassis_GetLastPwmRight(void);

int32_t AppChassis_GetLeftTotalTicks(void);
int32_t AppChassis_GetRightTotalTicks(void);

#endif
