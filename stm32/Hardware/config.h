#ifndef __CONFIG_H
#define __CONFIG_H

/* ========== 机械参数 ========== */
#define WHEEL_DIAMETER      0.048f       // 轮径(米)
#define WHEEL_BASE          0.128f       // 轮距(米)
#define ENCODER_PPR         11           // 编码器每圈脉冲数(MG310霍尔编码器)
#define GEAR_RATIO          20.0f        // 减速比 1:20
#define EMERGENCY_DIST      0.10f        // 紧急距离(米)

/* ========== PWM限幅 ========== */
#define PWM_MAX             100
#define PWM_MIN             -100

/* ========== 开环控制参数 ========== */
#define MAX_WHEEL_RPS       10.0f        // 最大轮速 (转/秒)，对应电机最高转速
#define MAX_PWM             100.0f       // 最大 PWM 值

/* ========== 运动学参数 ========== */
#define MAX_LINEAR_SPEED    0.8f         // 最大线速度 (m/s) - 保守值
#define MAX_ANGULAR_SPEED   13.0f        // 最大角速度 (rad/s) - 保守值


/* ========== 通信协议 ========== */
#define DOWNLINK_LEN        11               // 下行数据长度，不包含校验和
#define UPLINK_LEN          16               // 上行数据长度，包含校验和

#define WATCHDOG_MS   1000                // 看看门狗超时时间(毫秒)

#endif