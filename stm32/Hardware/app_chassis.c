#include "app_chassis.h"

#include "ChassisMap.h"
#include "ChassisSpeed.h"
#include "WheelPID.h"
#include "FlytSerial.h"
#include "app_imu.h"
#include "app_safety.h"
#include "config.h"

#define APP_CHASSIS_CMD_LINEAR_DEAD_MMPS     5
#define APP_CHASSIS_CMD_ANGULAR_DEAD_DPS     2.0f

static WheelPID_t g_pid_l;
static WheelPID_t g_pid_r;

/* 编码器累计值，发给飞腾派 */
static int32_t g_left_total_ticks = 0;
static int32_t g_right_total_ticks = 0;

/* 基础目标：由飞腾派 /cmd_vel 或蓝牙命令解算得到 */
static int32_t g_base_l_mmps = 0;
static int32_t g_base_r_mmps = 0;

/* 控制目标：在基础目标上叠加 MPU 航向修正 */
static int32_t g_ctrl_l_mmps = 0;
static int32_t g_ctrl_r_mmps = 0;

static int32_t g_target_limit_mmps = 350;
static int32_t g_min_control_speed_mmps = 150;
static int16_t g_pwm_limit = 85;

static int16_t g_last_dL = 0;
static int16_t g_last_dR = 0;
static int32_t g_last_actual_l = 0;
static int32_t g_last_actual_r = 0;
static int16_t g_last_pwm_l = 0;
static int16_t g_last_pwm_r = 0;

static int32_t clamp_i32(int32_t x, int32_t min_v, int32_t max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return x;
}

static int32_t abs_i32_local(int32_t x)
{
    return (x >= 0) ? x : -x;
}

static float abs_f_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int32_t apply_min_speed(int32_t v)
{
    if (v > 5 && v < g_min_control_speed_mmps)
    {
        return g_min_control_speed_mmps;
    }
    else if (v < -5 && v > -g_min_control_speed_mmps)
    {
        return -g_min_control_speed_mmps;
    }
    else if (v >= -5 && v <= 5)
    {
        return 0;
    }

    return v;
}

static void ApplyPwmLimit(void)
{
    g_pid_l.out_min = -g_pwm_limit;
    g_pid_l.out_max =  g_pwm_limit;

    g_pid_r.out_min = -g_pwm_limit;
    g_pid_r.out_max =  g_pwm_limit;
}

void AppChassis_Init(void)
{
    g_pid_l.Kp = 0.03f;
    g_pid_l.Ki = 0.02f;
    g_pid_l.Kd = 0.00f;
    g_pid_l.integral_min = -400.0f;
    g_pid_l.integral_max =  400.0f;
    g_pid_l.ff_gain = 0.027f;
    g_pid_l.min_start_pwm = 30;
    g_pid_l.max_step = 12;
    g_pid_l.no_reverse_brake = 1;

    g_pid_r.Kp = 0.03f;
    g_pid_r.Ki = 0.02f;
    g_pid_r.Kd = 0.00f;
    g_pid_r.integral_min = -400.0f;
    g_pid_r.integral_max =  400.0f;
    g_pid_r.ff_gain = 0.030f;
    g_pid_r.min_start_pwm = 30;
    g_pid_r.max_step = 12;
    g_pid_r.no_reverse_brake = 1;

    ApplyPwmLimit();

    WheelPID_Init(&g_pid_l);
    WheelPID_Init(&g_pid_r);
}

void AppChassis_ClearDelta(void)
{
    (void)ChassisMap_GetLeftDelta();
    (void)ChassisMap_GetRightDelta();
}

void AppChassis_ResetPid(void)
{
    WheelPID_Reset(&g_pid_l);
    WheelPID_Reset(&g_pid_r);
}

void AppChassis_Stop(void)
{
    g_base_l_mmps = 0;
    g_base_r_mmps = 0;
    g_ctrl_l_mmps = 0;
    g_ctrl_r_mmps = 0;
    AppImu_StopHeadingHold();

    AppChassis_ResetPid();

    ChassisMap_Stop();
    AppChassis_ClearDelta();
}

void AppChassis_UpdateTargetFromCmd(float linear_mps,
                                    float angular_radps,
                                    uint8_t estop_active)
{
    float v_mmps;
    float half_wheel_base_mm;
    int32_t left;
    int32_t right;
    float angular_dps;
    uint8_t should_hold;

    if (estop_active)
    {
        AppChassis_Stop();
        return;
    }

    v_mmps = linear_mps * 1000.0f;
    half_wheel_base_mm = WHEEL_BASE * 1000.0f * 0.5f;

    left = (int32_t)(v_mmps - angular_radps * half_wheel_base_mm);
    right = (int32_t)(v_mmps + angular_radps * half_wheel_base_mm);

    left = clamp_i32(left, -g_target_limit_mmps, g_target_limit_mmps);
    right = clamp_i32(right, -g_target_limit_mmps, g_target_limit_mmps);

    left = apply_min_speed(left);
    right = apply_min_speed(right);

    g_base_l_mmps = left;
    g_base_r_mmps = right;

    angular_dps = angular_radps * 57.29578f;

    /*
     * 只有“直线前进/后退”才启用航向保持。
     * 即：有线速度，角速度接近 0，且左右基础目标相等。
     */
    should_hold = 0;

    if (AppImu_GetGyroHoldEnable() &&
        abs_i32_local(g_base_l_mmps) > APP_CHASSIS_CMD_LINEAR_DEAD_MMPS &&
        g_base_l_mmps == g_base_r_mmps &&
        abs_f_local(angular_dps) < APP_CHASSIS_CMD_ANGULAR_DEAD_DPS)
    {
        should_hold = 1;
    }

    AppImu_UpdateHeadingHoldState(should_hold);
}

uint8_t AppChassis_SetBluetoothWheelTarget(int32_t left_mmps,
                                           int32_t right_mmps,
                                           uint8_t estop_active)
{
    if (estop_active)
    {
        AppChassis_Stop();
        return 0;
    }

    left_mmps = clamp_i32(left_mmps, -g_target_limit_mmps, g_target_limit_mmps);
    right_mmps = clamp_i32(right_mmps, -g_target_limit_mmps, g_target_limit_mmps);

    left_mmps = apply_min_speed(left_mmps);
    right_mmps = apply_min_speed(right_mmps);

    g_base_l_mmps = left_mmps;
    g_base_r_mmps = right_mmps;

    return 1;
}

void AppChassis_BuildControlTarget(void)
{
    g_ctrl_l_mmps = g_base_l_mmps;
    g_ctrl_r_mmps = g_base_r_mmps;

    (void)AppImu_ApplyHeadingCorrection(&g_ctrl_l_mmps,
                                        &g_ctrl_r_mmps,
                                        g_target_limit_mmps);
}

void AppChassis_ControlAndUplinkStep(uint8_t estop_active)
{
    int16_t dL;
    int16_t dR;
    int32_t actual_l;
    int32_t actual_r;
    int16_t pwm_l;
    int16_t pwm_r;
    uint8_t safety_result;

    AppImu_Update();

    if (estop_active)
    {
        AppChassis_Stop();
    }

    AppChassis_BuildControlTarget();

    safety_result = AppSafety_ApplyUltrasonicToTargets(&g_ctrl_l_mmps, &g_ctrl_r_mmps);
    if (safety_result & APP_SAFETY_RESULT_STOP)
    {
        AppImu_StopHeadingHold();
        AppChassis_ResetPid();
    }
    else if (safety_result & APP_SAFETY_RESULT_LIMIT)
    {
        AppImu_ClearLastYawCorr();
    }

    dL = ChassisMap_GetLeftDelta();
    dR = ChassisMap_GetRightDelta();

    g_left_total_ticks += dL;
    g_right_total_ticks += dR;

    actual_l = ChassisSpeed_DeltaToMmps(dL, 50);
    actual_r = ChassisSpeed_DeltaToMmps(dR, 50);

    pwm_l = WheelPID_Update(&g_pid_l, g_ctrl_l_mmps, actual_l, 50);
    pwm_r = WheelPID_Update(&g_pid_r, g_ctrl_r_mmps, actual_r, 50);

    if (estop_active)
    {
        pwm_l = 0;
        pwm_r = 0;
    }

    ChassisMap_SetWheelPWM(pwm_l, pwm_r);

    g_last_dL = dL;
    g_last_dR = dR;
    g_last_actual_l = actual_l;
    g_last_actual_r = actual_r;
    g_last_pwm_l = pwm_l;
    g_last_pwm_r = pwm_r;

    FlytSerial_SendUplink(g_left_total_ticks,
                          g_right_total_ticks,
                          AppImu_GetGyroZDps(),
                          AppSafety_BuildStatusByte(estop_active));
}

void AppChassis_SetKpLeft(float v) { g_pid_l.Kp = v; }
void AppChassis_SetKpRight(float v) { g_pid_r.Kp = v; }
void AppChassis_SetKpBoth(float v) { g_pid_l.Kp = v; g_pid_r.Kp = v; }

void AppChassis_SetKiLeft(float v) { g_pid_l.Ki = v; }
void AppChassis_SetKiRight(float v) { g_pid_r.Ki = v; }
void AppChassis_SetKiBoth(float v) { g_pid_l.Ki = v; g_pid_r.Ki = v; }

void AppChassis_SetFFLeft(float v) { g_pid_l.ff_gain = v; }
void AppChassis_SetFFRight(float v) { g_pid_r.ff_gain = v; }

void AppChassis_SetStartLeft(int16_t v) { g_pid_l.min_start_pwm = v; }
void AppChassis_SetStartRight(int16_t v) { g_pid_r.min_start_pwm = v; }

void AppChassis_SetMinControlSpeed(int32_t v) { g_min_control_speed_mmps = v; }
void AppChassis_SetTargetLimit(int32_t v) { g_target_limit_mmps = v; }
void AppChassis_SetPwmLimit(int16_t v) { g_pwm_limit = v; ApplyPwmLimit(); }
void AppChassis_SetStep(int16_t v) { g_pid_l.max_step = v; g_pid_r.max_step = v; }

float AppChassis_GetKpLeft(void) { return g_pid_l.Kp; }
float AppChassis_GetKpRight(void) { return g_pid_r.Kp; }
float AppChassis_GetKiLeft(void) { return g_pid_l.Ki; }
float AppChassis_GetKiRight(void) { return g_pid_r.Ki; }
float AppChassis_GetFFLeft(void) { return g_pid_l.ff_gain; }
float AppChassis_GetFFRight(void) { return g_pid_r.ff_gain; }
int16_t AppChassis_GetStartLeft(void) { return g_pid_l.min_start_pwm; }
int16_t AppChassis_GetStartRight(void) { return g_pid_r.min_start_pwm; }
int32_t AppChassis_GetMinControlSpeed(void) { return g_min_control_speed_mmps; }
int32_t AppChassis_GetTargetLimit(void) { return g_target_limit_mmps; }
int16_t AppChassis_GetPwmLimit(void) { return g_pwm_limit; }
int16_t AppChassis_GetStep(void) { return g_pid_l.max_step; }

int32_t AppChassis_GetBaseLeftMmps(void) { return g_base_l_mmps; }
int32_t AppChassis_GetBaseRightMmps(void) { return g_base_r_mmps; }
int32_t AppChassis_GetCtrlLeftMmps(void) { return g_ctrl_l_mmps; }
int32_t AppChassis_GetCtrlRightMmps(void) { return g_ctrl_r_mmps; }

int16_t AppChassis_GetLastDeltaLeft(void) { return g_last_dL; }
int16_t AppChassis_GetLastDeltaRight(void) { return g_last_dR; }
int32_t AppChassis_GetLastActualLeftMmps(void) { return g_last_actual_l; }
int32_t AppChassis_GetLastActualRightMmps(void) { return g_last_actual_r; }
int16_t AppChassis_GetLastPwmLeft(void) { return g_last_pwm_l; }
int16_t AppChassis_GetLastPwmRight(void) { return g_last_pwm_r; }

int32_t AppChassis_GetLeftTotalTicks(void) { return g_left_total_ticks; }
int32_t AppChassis_GetRightTotalTicks(void) { return g_right_total_ticks; }
