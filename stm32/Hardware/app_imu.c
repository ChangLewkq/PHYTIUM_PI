#include "app_imu.h"
#include "MPU6050.h"
#include "BlueSerial.h"
#include "Delay.h"

#define APP_IMU_CALIB_SAMPLES      500
#define APP_IMU_CALIB_DELAY_MS     4

static uint16_t g_ctrl_period_ms = 50;

static uint8_t g_mpu_id = 0;
static uint8_t g_mpu_online = 0;
static uint16_t g_mpu_retry_ms = 0;
static float g_gyro_z_offset_dps = 0.0f;
static float g_gyro_z_dps = 0.0f;
static float g_yaw_deg = 0.0f;

/* 航向保持 */
static uint8_t g_gyro_hold_enable = 1;
static uint8_t g_heading_hold_active = 0;
static float g_yaw_ref_deg = 0.0f;

static float g_yaw_kp = 3.0f;       /* mm/s per deg */
static float g_yaw_kd = 0.5f;       /* mm/s per deg/s */
static int32_t g_yaw_corr_limit = 50;
static int32_t g_last_yaw_corr = 0;

static int32_t clamp_i32_local(int32_t x, int32_t min_v, int32_t max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return x;
}

static float normalize_angle_deg_local(float a)
{
    while (a > 180.0f)
    {
        a -= 360.0f;
    }

    while (a < -180.0f)
    {
        a += 360.0f;
    }

    return a;
}

static int32_t f_to_i100_local(float x)
{
    if (x >= 0.0f)
    {
        return (int32_t)(x * 100.0f + 0.5f);
    }
    else
    {
        return (int32_t)(x * 100.0f - 0.5f);
    }
}

void AppImu_Init(uint16_t ctrl_period_ms)
{
    if (ctrl_period_ms > 0)
    {
        g_ctrl_period_ms = ctrl_period_ms;
    }

    MPU6050_Init();

    g_mpu_id = MPU6050_GetID();
    g_mpu_online = MPU6050_IsValidID(g_mpu_id);
    g_mpu_retry_ms = 0;

    g_gyro_z_offset_dps = 0.0f;
    g_gyro_z_dps = 0.0f;
    g_yaw_deg = 0.0f;
    g_yaw_ref_deg = 0.0f;
    g_heading_hold_active = 0;
    g_last_yaw_corr = 0;
}

void AppImu_CalibrateGyroZ(void)
{
    int32_t i;
    float sum = 0.0f;

    BlueSerial_Printf("CAL start, keep still...\r\n");

    g_mpu_id = MPU6050_GetID();
    g_mpu_online = MPU6050_IsValidID(g_mpu_id);

    if (!g_mpu_online)
    {
        g_gyro_z_offset_dps = 0.0f;
        g_gyro_z_dps = 0.0f;
        g_yaw_deg = 0.0f;
        g_yaw_ref_deg = 0.0f;
        g_heading_hold_active = 0;
        g_last_yaw_corr = 0;

        BlueSerial_Printf("CAL skip, MPU ID=0x%02X\r\n", g_mpu_id);
        return;
    }

    for (i = 0; i < APP_IMU_CALIB_SAMPLES; i++)
    {
        sum += MPU6050_GetGyroZ();
        Delay_ms(APP_IMU_CALIB_DELAY_MS);
    }

    g_gyro_z_offset_dps = sum / (float)APP_IMU_CALIB_SAMPLES;
    g_yaw_deg = 0.0f;
    g_yaw_ref_deg = 0.0f;
    g_heading_hold_active = 0;
    g_last_yaw_corr = 0;

    BlueSerial_Printf("CAL done offset_x100=%ld\r\n",
                      (long)f_to_i100_local(g_gyro_z_offset_dps));
}

void AppImu_Update(void)
{
    float raw;
    float dt;

    /*
     * 如果 MPU 当前不在线，不再用 0xFF 读数参与 yaw 积分。
     * 同时每约 1s 尝试重新初始化一次，便于热恢复。
     */
    if (!g_mpu_online)
    {
        g_gyro_z_dps = 0.0f;
        g_heading_hold_active = 0;
        g_last_yaw_corr = 0;

        g_mpu_retry_ms += g_ctrl_period_ms;

        if (g_mpu_retry_ms >= 1000)
        {
            g_mpu_retry_ms = 0;
            MPU6050_Init();
            g_mpu_id = MPU6050_GetID();
            g_mpu_online = MPU6050_IsValidID(g_mpu_id);
        }

        return;
    }

    raw = MPU6050_GetGyroZ();
    g_gyro_z_dps = raw - g_gyro_z_offset_dps;

    dt = (float)g_ctrl_period_ms / 1000.0f;
    g_yaw_deg += g_gyro_z_dps * dt;
    g_yaw_deg = normalize_angle_deg_local(g_yaw_deg);
}

uint8_t AppImu_GetID(void)
{
    return g_mpu_id;
}

uint8_t AppImu_IsOnline(void)
{
    return g_mpu_online;
}

float AppImu_GetGyroZOffsetDps(void)
{
    return g_gyro_z_offset_dps;
}

float AppImu_GetGyroZDps(void)
{
    return g_gyro_z_dps;
}

float AppImu_GetYawDeg(void)
{
    return g_yaw_deg;
}

void AppImu_ResetYaw(void)
{
    g_yaw_deg = 0.0f;
    g_yaw_ref_deg = 0.0f;
    g_heading_hold_active = 0;
    g_last_yaw_corr = 0;
}

void AppImu_SetGyroHoldEnable(uint8_t enable)
{
    g_gyro_hold_enable = enable ? 1 : 0;

    if (!g_gyro_hold_enable)
    {
        AppImu_StopHeadingHold();
    }
}

uint8_t AppImu_GetGyroHoldEnable(void)
{
    return g_gyro_hold_enable;
}

void AppImu_UpdateHeadingHoldState(uint8_t should_hold)
{
    if (should_hold)
    {
        if (!g_heading_hold_active)
        {
            g_heading_hold_active = 1;
            g_yaw_ref_deg = g_yaw_deg;
            g_last_yaw_corr = 0;
        }
    }
    else
    {
        AppImu_StopHeadingHold();
    }
}

void AppImu_StopHeadingHold(void)
{
    g_heading_hold_active = 0;
    g_last_yaw_corr = 0;
}

uint8_t AppImu_GetHeadingHoldActive(void)
{
    return g_heading_hold_active;
}

void AppImu_SetYawKp(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    g_yaw_kp = kp;
}

void AppImu_SetYawKd(float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    g_yaw_kd = kd;
}

void AppImu_SetYawCorrLimit(int32_t limit)
{
    if (limit < 0) limit = 0;
    g_yaw_corr_limit = limit;
}

float AppImu_GetYawKp(void)
{
    return g_yaw_kp;
}

float AppImu_GetYawKd(void)
{
    return g_yaw_kd;
}

int32_t AppImu_GetYawCorrLimit(void)
{
    return g_yaw_corr_limit;
}

int32_t AppImu_GetLastYawCorr(void)
{
    return g_last_yaw_corr;
}

void AppImu_ClearLastYawCorr(void)
{
    g_last_yaw_corr = 0;
}

uint8_t AppImu_ApplyHeadingCorrection(int32_t *left_mmps,
                                      int32_t *right_mmps,
                                      int32_t target_limit_mmps)
{
    float yaw_error;
    float corr_f;
    int32_t corr;

    if (left_mmps == 0 || right_mmps == 0)
    {
        return 0;
    }

    g_last_yaw_corr = 0;

    if (!g_mpu_online)
    {
        return 0;
    }

    if (!(g_heading_hold_active && g_gyro_hold_enable))
    {
        return 0;
    }

    yaw_error = normalize_angle_deg_local(g_yaw_ref_deg - g_yaw_deg);

    /*
     * corr > 0:
     * left = base - corr
     * right = base + corr
     */
    corr_f = g_yaw_kp * yaw_error - g_yaw_kd * g_gyro_z_dps;

    if (corr_f > (float)g_yaw_corr_limit)
    {
        corr_f = (float)g_yaw_corr_limit;
    }
    else if (corr_f < -(float)g_yaw_corr_limit)
    {
        corr_f = -(float)g_yaw_corr_limit;
    }

    corr = (int32_t)corr_f;

    *left_mmps = *left_mmps - corr;
    *right_mmps = *right_mmps + corr;

    *left_mmps = clamp_i32_local(*left_mmps, -target_limit_mmps, target_limit_mmps);
    *right_mmps = clamp_i32_local(*right_mmps, -target_limit_mmps, target_limit_mmps);

    g_last_yaw_corr = corr;

    return 1;
}
