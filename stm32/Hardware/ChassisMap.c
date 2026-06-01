#include "ChassisMap.h"
#include "Motor.h"
#include "Encoder.h"

static int8_t clamp_pwm_i8(int16_t pwm)
{
    if (pwm > 100) return 100;
    if (pwm < -100) return -100;
    return (int8_t)pwm;
}

/*
 * 第一、二轮实测结论：
 *
 * Motor_SetPWM(1, x) -> 物理右轮
 *   x > 0 时，物理右轮反转
 *   x < 0 时，物理右轮正转
 *
 * Motor_SetPWM(2, x) -> 物理左轮
 *   x > 0 时，物理左轮正转
 *   x < 0 时，物理左轮反转
 *
 * 因此：
 * 物理左轮正转：Motor_SetPWM(2, +pwm)
 * 物理右轮正转：Motor_SetPWM(1, -pwm)
 */
void ChassisMap_SetWheelPWM(int16_t left_pwm, int16_t right_pwm)
{
    int8_t l = clamp_pwm_i8(left_pwm);
    int8_t r = clamp_pwm_i8(right_pwm);

    Motor_SetPWM(1, -r);
    Motor_SetPWM(2, -l);
}

void ChassisMap_Stop(void)
{
    ChassisMap_SetWheelPWM(0, 0);
}

/*
 * 第二轮实测结论：
 *
 * Encoder_Get(1) -> 物理右轮编码器 raw1
 *   右轮正转时 raw1 > 0
 *   右轮反转时 raw1 < 0
 *
 * Encoder_Get(2) -> 物理左轮编码器 raw2
 *   左轮正转时 raw2 > 0
 *   左轮反转时 raw2 < 0
 *
 * 因此编码器不需要取反，只需要交换左右物理含义。
 */
int16_t ChassisMap_GetLeftDelta(void)
{
    return Encoder_Get(2);
}

int16_t ChassisMap_GetRightDelta(void)
{
    return Encoder_Get(1);
}
