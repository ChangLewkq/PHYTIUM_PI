#include "app_safety.h"
#include "ultrasonic.h"
#include "FlytProtocol.h"

/*
 * app_safety.c
 *
 * 第一阶段模块化：只把 STM32 本地安全状态、超声波测距/限速、
 * 飞腾派心跳状态、上行 safety_flags 从 main.c 中拆出。
 *
 * 注意：
 * 1. 不改变现有 11/16 字节飞腾派串口协议。
 * 2. 不改变原有超声波安全策略。
 * 3. 不直接操作 PID，不直接操作 PWM。
 * 4. 如果触发 <=10cm 前进停车，本模块只返回 STOP 标志，
 *    由 main.c 负责重置 PID / 航向保持。
 */

static uint8_t  s_ultra_enable = 0;
static uint16_t s_ultra_timer_ms = 0;
static float    s_ultra_dist_m = APP_SAFETY_ULTRA_VALID_MAX_M;
static uint8_t  s_ultra_valid = 0;
static uint8_t  s_ultra_limited = 0;
static uint8_t  s_ultra_stop = 0;

/* 上电默认认为飞腾派未连接/心跳超时，收到合法下行包后由 main.c 清零 */
static uint8_t  s_pi_timeout = 1;

static uint8_t AppSafety_IsForwardCommand(int32_t left_mmps, int32_t right_mmps)
{
    /*
     * 对两轮差速车来说，左右轮都为正时基本就是整体前进。
     * 原地旋转/后退不拦截，避免近距离时无法转向或后退脱困。
     */
    return (left_mmps > 0 && right_mmps > 0) ? 1 : 0;
}

void AppSafety_Init(void)
{
    s_ultra_enable = 1;
    s_ultra_timer_ms = 0;
    s_ultra_dist_m = APP_SAFETY_ULTRA_VALID_MAX_M;
    s_ultra_valid = 0;
    s_ultra_limited = 0;
    s_ultra_stop = 0;
    s_pi_timeout = 1;
}

void AppSafety_SetPiTimeout(uint8_t timeout)
{
    s_pi_timeout = timeout ? 1 : 0;
}

uint8_t AppSafety_GetPiTimeout(void)
{
    return s_pi_timeout;
}

void AppSafety_SetUltrasonicEnable(uint8_t enable)
{
    s_ultra_enable = enable ? 1 : 0;

    if (!s_ultra_enable)
    {
        s_ultra_dist_m = APP_SAFETY_ULTRA_VALID_MAX_M;
        s_ultra_valid = 0;
        s_ultra_limited = 0;
        s_ultra_stop = 0;
        s_ultra_timer_ms = 0;
    }
}

uint8_t AppSafety_GetUltrasonicEnable(void)
{
    return s_ultra_enable;
}

void AppSafety_Update(uint16_t dt_ms)
{
    float d;

    if (!s_ultra_enable)
    {
        s_ultra_dist_m = APP_SAFETY_ULTRA_VALID_MAX_M;
        s_ultra_valid = 0;
        s_ultra_limited = 0;
        s_ultra_stop = 0;
        return;
    }

    s_ultra_timer_ms += dt_ms;

    if (s_ultra_timer_ms < APP_SAFETY_ULTRA_PERIOD_MS)
    {
        return;
    }

    s_ultra_timer_ms = 0;

    d = Ultrasonic_Read();

    if (d >= APP_SAFETY_ULTRA_VALID_MIN_M && d <= APP_SAFETY_ULTRA_VALID_MAX_M)
    {
        s_ultra_dist_m = d;
        s_ultra_valid = 1;
    }
    else
    {
        /*
         * 超时或异常时，不直接停车。
         * 原因：HC-SR04 偶发超时较常见，误触发会影响正常控制。
         * 真正的飞腾派失联由心跳超时负责停车。
         */
        s_ultra_dist_m = APP_SAFETY_ULTRA_VALID_MAX_M;
        s_ultra_valid = 0;
    }
}

uint8_t AppSafety_ApplyUltrasonicToTargets(int32_t *left_mmps, int32_t *right_mmps)
{
    uint8_t result = APP_SAFETY_RESULT_NONE;

    s_ultra_limited = 0;
    s_ultra_stop = 0;

    if (left_mmps == 0 || right_mmps == 0)
    {
        return result;
    }

    if (!s_ultra_enable || !s_ultra_valid)
    {
        return result;
    }

    /*
     * <=10cm：只禁止继续前进。
     * 允许后退和原地转向，防止车停在障碍物前无法脱困。
     */
    if (s_ultra_dist_m <= APP_SAFETY_ULTRA_STOP_DIST_M)
    {
        if (AppSafety_IsForwardCommand(*left_mmps, *right_mmps))
        {
            *left_mmps = 0;
            *right_mmps = 0;

            s_ultra_limited = 1;
            s_ultra_stop = 1;
            result |= APP_SAFETY_RESULT_LIMIT;
            result |= APP_SAFETY_RESULT_STOP;
        }

        return result;
    }

    /*
     * 10cm~25cm：只限制前进速度到 APP_SAFETY_ULTRA_LIMIT_MMPS。
     * 后退和原地转向不限制，方便脱困。
     */
    if (s_ultra_dist_m <= APP_SAFETY_ULTRA_SLOW_DIST_M)
    {
        if (AppSafety_IsForwardCommand(*left_mmps, *right_mmps))
        {
            if (*left_mmps > APP_SAFETY_ULTRA_LIMIT_MMPS)
            {
                *left_mmps = APP_SAFETY_ULTRA_LIMIT_MMPS;
                s_ultra_limited = 1;
            }

            if (*right_mmps > APP_SAFETY_ULTRA_LIMIT_MMPS)
            {
                *right_mmps = APP_SAFETY_ULTRA_LIMIT_MMPS;
                s_ultra_limited = 1;
            }

            if (s_ultra_limited)
            {
                result |= APP_SAFETY_RESULT_LIMIT;
            }
        }
    }

    return result;
}

uint8_t AppSafety_BuildStatusByte(uint8_t estop_active)
{
    uint8_t status = 0;

    if (estop_active)
    {
        status |= FLYT_STATUS_ESTOP;
    }

    if (s_ultra_limited)
    {
        status |= FLYT_STATUS_ULTRA_SLOW;
    }

    if (s_ultra_stop)
    {
        status |= FLYT_STATUS_ULTRA_STOP;
    }

    if (s_pi_timeout)
    {
        status |= FLYT_STATUS_PI_TIMEOUT;
    }

    return status;
}

float AppSafety_GetUltraDistM(void)
{
    return s_ultra_dist_m;
}

uint8_t AppSafety_GetUltraValid(void)
{
    return s_ultra_valid;
}

uint8_t AppSafety_GetUltraLimited(void)
{
    return s_ultra_limited;
}

uint8_t AppSafety_GetUltraStop(void)
{
    return s_ultra_stop;
}
