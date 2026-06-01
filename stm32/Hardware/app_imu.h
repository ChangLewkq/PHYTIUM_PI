#ifndef __APP_IMU_H
#define __APP_IMU_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * AppImu
 *
 * 功能：
 * 1. 初始化 MPU6050
 * 2. Gyro_z 零偏校准
 * 3. gyro_z / yaw 更新
 * 4. 直线航向保持状态管理
 * 5. 根据 yaw 误差生成左右轮差速修正
 *
 * 说明：
 * - 本模块暂时仍使用裸机 Delay_ms 进行校准延时。
 * - 后续迁移 RT-Thread 时，AppImu_Update() 可直接放入 imu_task。
 */

void AppImu_Init(uint16_t ctrl_period_ms);
void AppImu_CalibrateGyroZ(void);
void AppImu_Update(void);

uint8_t AppImu_GetID(void);
uint8_t AppImu_IsOnline(void);

float AppImu_GetGyroZOffsetDps(void);
float AppImu_GetGyroZDps(void);
float AppImu_GetYawDeg(void);

void AppImu_ResetYaw(void);

void AppImu_SetGyroHoldEnable(uint8_t enable);
uint8_t AppImu_GetGyroHoldEnable(void);

void AppImu_UpdateHeadingHoldState(uint8_t should_hold);
void AppImu_StopHeadingHold(void);
uint8_t AppImu_GetHeadingHoldActive(void);

void AppImu_SetYawKp(float kp);
void AppImu_SetYawKd(float kd);
void AppImu_SetYawCorrLimit(int32_t limit);

float AppImu_GetYawKp(void);
float AppImu_GetYawKd(void);
int32_t AppImu_GetYawCorrLimit(void);
int32_t AppImu_GetLastYawCorr(void);
void AppImu_ClearLastYawCorr(void);

/*
 * 对左右轮目标速度叠加航向修正。
 *
 * 参数：
 * left_mmps / right_mmps：输入输出，单位 mm/s
 * target_limit_mmps：左右轮目标限幅，单位 mm/s
 *
 * 返回：
 * 1：本次应用了航向修正
 * 0：未应用航向修正
 */
uint8_t AppImu_ApplyHeadingCorrection(int32_t *left_mmps,
                                      int32_t *right_mmps,
                                      int32_t target_limit_mmps);

#endif
