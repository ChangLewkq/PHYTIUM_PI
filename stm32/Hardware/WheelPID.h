#ifndef __WHEEL_PID_H
#define __WHEEL_PID_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * 左/右轮速度 PID
 * target / actual: mm/s
 * output: PWM, -100~100
 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float integral_min;
    float integral_max;

    int32_t last_error;
    int32_t last_actual;

    int16_t out_min;
    int16_t out_max;

    float ff_gain;          /* 前馈系数：PWM / (mm/s) */
    int16_t min_start_pwm;  /* 非零目标时的基础 PWM */

    int16_t last_output;    /* 用于输出斜率限制 */
    int16_t max_step;       /* 每个控制周期 PWM 最大变化量 */

    uint8_t no_reverse_brake; /* 目标为正时不允许输出负PWM；目标为负时不允许输出正PWM */
} WheelPID_t;

void WheelPID_Init(WheelPID_t *pid);
void WheelPID_Reset(WheelPID_t *pid);

int16_t WheelPID_Update(WheelPID_t *pid,
                        int32_t target_mmps,
                        int32_t actual_mmps,
                        uint16_t dt_ms);

#endif
