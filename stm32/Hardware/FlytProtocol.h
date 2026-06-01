#ifndef __FLYT_PROTOCOL_H
#define __FLYT_PROTOCOL_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * 飞腾派 <-> STM32 二进制协议
 *
 * 下行：飞腾派 -> STM32, 11 bytes
 * [0] 0xAA
 * [1] 0x55
 * [2:5]  float linear_mps, 小端
 * [6:9]  float angular_radps, 小端
 * [10]   XOR checksum of bytes 0~9
 *
 * 上行：STM32 -> 飞腾派, 16 bytes
 * [0] 0xBB
 * [1] 0x66
 * [2:5]  int32 left_total_ticks, 小端
 * [6:9]  int32 right_total_ticks, 小端
 * [10:13] float gyro_z_dps, 小端，MPU6050 Z轴角速度，单位 deg/s
 * [14] uint8 status/safety_flags, 见 FLYT_STATUS_xxx
 * [15] XOR checksum of bytes 0~14
 */

#define FLYT_DOWNLINK_LEN    11
#define FLYT_UPLINK_LEN      16

#define FLYT_DOWN_STX1       0xAA
#define FLYT_DOWN_STX2       0x55

#define FLYT_UP_STX1         0xBB
#define FLYT_UP_STX2         0x66

/* ========== STM32 -> 飞腾派 status/safety_flags ==========
 * 上行帧第 [14] 字节，不再只是 collision，而是安全状态位。
 *
 * 注意：
 * 旧版飞腾派端 bool(status) 仍然兼容：
 * 只要 status != 0，就表示 STM32 端存在安全/异常状态。
 */
#define FLYT_STATUS_ESTOP          0x01  /* 急停/蓝牙急停 */
#define FLYT_STATUS_ULTRA_SLOW     0x02  /* 超声波前进限速 */
#define FLYT_STATUS_ULTRA_STOP     0x04  /* 超声波禁止继续前进 */
#define FLYT_STATUS_PI_TIMEOUT     0x08  /* 飞腾派心跳超时 */
#define FLYT_STATUS_MPU_ERROR      0x10  /* MPU6050 异常，预留 */
#define FLYT_STATUS_ENCODER_ERR    0x20  /* 编码器异常，预留 */
#define FLYT_STATUS_BT_ACTIVE      0x40  /* 蓝牙控制/调试活动，预留 */
#define FLYT_STATUS_RESERVED       0x80

typedef struct
{
    float linear_mps;
    float angular_radps;
    uint8_t valid;
} FlytCmd_t;

uint8_t FlytProtocol_Xor(const uint8_t *buf, uint16_t len);
uint8_t FlytProtocol_ParseDownlink(const uint8_t *frame, FlytCmd_t *cmd);
void FlytProtocol_PackUplink(int32_t left_total_ticks,
                             int32_t right_total_ticks,
                             float gyro_z_dps,
                             uint8_t status,
                             uint8_t out_frame[FLYT_UPLINK_LEN]);

#endif
