#include "WheelPID.h"

static int32_t abs_i32(int32_t x)
{
    return (x >= 0) ? x : -x;
}

static int16_t clamp_i16(int32_t x, int16_t min_v, int16_t max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return (int16_t)x;
}

static int16_t slew_limit(int16_t target, int16_t last, int16_t max_step)
{
    int16_t diff;

    if (max_step <= 0)
    {
        return target;
    }

    diff = target - last;

    if (diff > max_step)
    {
        return last + max_step;
    }
    else if (diff < -max_step)
    {
        return last - max_step;
    }

    return target;
}

void WheelPID_Init(WheelPID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0;
    pid->last_actual = 0;
    pid->last_output = 0;
}

void WheelPID_Reset(WheelPID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0;
    pid->last_actual = 0;
    pid->last_output = 0;
}

int16_t WheelPID_Update(WheelPID_t *pid,
                        int32_t target_mmps,
                        int32_t actual_mmps,
                        uint16_t dt_ms)
{
    float dt;
    float error;
    float derivative;
    float ff;
    float out_f;
    int32_t out_i;
    int16_t out;

    if (dt_ms == 0)
    {
        return 0;
    }

    /*
     * 异常速度保护：
     * 低速调试时，如果单个100ms采样算出几千 mm/s，通常是打滑、拖拽或编码器瞬时异常。
     * 不让这个异常值直接把积分和输出打爆。
     */
    if (actual_mmps > 1000)
    {
        actual_mmps = 1000;
    }
    else if (actual_mmps < -1000)
    {
        actual_mmps = -1000;
    }

    if (abs_i32(target_mmps) < 5)
    {
        WheelPID_Reset(pid);
        return 0;
    }

    dt = (float)dt_ms / 1000.0f;
    error = (float)(target_mmps - actual_mmps);

    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    derivative = ((float)(error - (float)pid->last_error)) / dt;

    if (target_mmps > 0)
    {
        ff = (float)pid->min_start_pwm + pid->ff_gain * (float)target_mmps;
    }
    else
    {
        ff = -(float)pid->min_start_pwm + pid->ff_gain * (float)target_mmps;
    }

    out_f = ff
          + pid->Kp * error
          + pid->Ki * pid->integral
          + pid->Kd * derivative;

    out_i = (int32_t)(out_f >= 0.0f ? out_f + 0.5f : out_f - 0.5f);

    /*
     * 关键保护：
     * 当前落地低速调试阶段不允许“为了减速而反打电机”。
     * 之前左轮出现过目标正向、PWM反向，导致低速更乱。
     */
    if (pid->no_reverse_brake)
    {
        if (target_mmps > 0 && out_i < 0)
        {
            out_i = 0;
        }
        else if (target_mmps < 0 && out_i > 0)
        {
            out_i = 0;
        }
    }

    out = clamp_i16(out_i, pid->out_min, pid->out_max);
    out = slew_limit(out, pid->last_output, pid->max_step);

    pid->last_error = (int32_t)error;
    pid->last_actual = actual_mmps;
    pid->last_output = out;

    return out;
}
