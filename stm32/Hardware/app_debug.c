#include "app_debug.h"

#include "BlueSerial.h"
#include "app_chassis.h"
#include "app_health.h"
#include "app_imu.h"
#include "app_status.h"
#include "app_safety.h"
#include "Encoder.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

static uint16_t g_ctrl_period_ms = 50;
static uint16_t g_report_period_ms = 500;
static uint16_t g_cmd_watchdog_ms = 500;
static uint16_t g_bt_report_ms = 0;

static uint8_t g_bt_debug_enable = 0;
static uint8_t g_bt_estop = 0;

static int32_t f_to_i10(float x)
{
    if (x >= 0.0f)
    {
        return (int32_t)(x * 10.0f + 0.5f);
    }
    else
    {
        return (int32_t)(x * 10.0f - 0.5f);
    }
}

static int32_t f_to_i100(float x)
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

static int32_t ReadNumberAfter(const char *cmd, uint8_t offset)
{
    return atoi(cmd + offset);
}

static uint8_t SetBluetoothMotionTarget(int32_t left_mmps, int32_t right_mmps)
{
    uint8_t ok;

    ok = AppChassis_SetBluetoothWheelTarget(left_mmps, right_mmps, g_bt_estop);

    if (ok)
    {
        BlueSerial_Printf("BTMOVE L=%ld R=%ld hold=%dms\r\n",
                          (long)AppChassis_GetBaseLeftMmps(),
                          (long)AppChassis_GetBaseRightMmps(),
                          g_cmd_watchdog_ms);
        return 1;
    }
    else
    {
        BlueSerial_Printf("ESTOP active\r\n");
        return 0;
    }
}

void AppDebug_Init(uint16_t ctrl_period_ms,
                   uint16_t report_period_ms,
                   uint16_t cmd_watchdog_ms)
{
    g_ctrl_period_ms = ctrl_period_ms;
    g_report_period_ms = report_period_ms;
    g_cmd_watchdog_ms = cmd_watchdog_ms;
    g_bt_report_ms = 0;
    g_bt_debug_enable = 0;
    g_bt_estop = 0;

    BlueSerial_Init();
}

uint8_t AppDebug_GetEstop(void)
{
    return g_bt_estop;
}

uint8_t AppDebug_GetDebugEnable(void)
{
    return g_bt_debug_enable;
}

void AppDebug_PrintHelp(void)
{
    BlueSerial_Printf("\r\nFLYT UART + MPU HOLD OFFICIAL\r\n");
    BlueSerial_Printf("[H] [STAT] [TASK] [DBG1] [DBG0] [S] [E1] [E0]\r\n");
    BlueSerial_Printf("[TASK] shows thread health, WDG shows feed permission in [STAT]\r\n");
    BlueSerial_Printf("[P3] [I2] [PL3] [PR3] [IL2] [IR2]\r\n");
    BlueSerial_Printf("[FL27] [FR30] [SL30] [SR30]\r\n");
    BlueSerial_Printf("[MS150] [LIM350] [OUT85] [STEP12]\r\n");
    BlueSerial_Printf("[CAL] [YAW0] [GYRO1] [GYRO0] [YKP30] [YKD5] [YC50]\r\n");
    BlueSerial_Printf("[US1] [US0]  US: 25~10cm limit, <=10cm block forward\r\n");
    BlueSerial_Printf("[F180] [B120] [L120] [R120] bluetooth motion test\r\n");
}

void AppDebug_PrintStatus(void)
{
    BlueSerial_Printf("\r\nPARAM\r\n");
    BlueSerial_Printf("Kp L=%ld R=%ld /100 Ki L=%ld R=%ld /100\r\n",
                      (long)(AppChassis_GetKpLeft() * 100.0f + 0.5f),
                      (long)(AppChassis_GetKpRight() * 100.0f + 0.5f),
                      (long)(AppChassis_GetKiLeft() * 100.0f + 0.5f),
                      (long)(AppChassis_GetKiRight() * 100.0f + 0.5f));

    BlueSerial_Printf("FF L=%ld R=%ld /1000 START L=%d R=%d\r\n",
                      (long)(AppChassis_GetFFLeft() * 1000.0f + 0.5f),
                      (long)(AppChassis_GetFFRight() * 1000.0f + 0.5f),
                      AppChassis_GetStartLeft(),
                      AppChassis_GetStartRight());

    BlueSerial_Printf("MS=%ld LIM=%ld OUT=%d STEP=%d\r\n",
                      (long)AppChassis_GetMinControlSpeed(),
                      (long)AppChassis_GetTargetLimit(),
                      AppChassis_GetPwmLimit(),
                      AppChassis_GetStep());

    BlueSerial_Printf("MPU ID=0x%02X IMU_OK=%d off_x100=%ld yaw_x10=%ld gz_x100=%ld\r\n",
                      AppImu_GetID(),
                      AppImu_IsOnline(),
                      (long)f_to_i100(AppImu_GetGyroZOffsetDps()),
                      (long)f_to_i10(AppImu_GetYawDeg()),
                      (long)f_to_i100(AppImu_GetGyroZDps()));

    BlueSerial_Printf("GYRO=%d HOLD=%d YKP_x10=%ld YKD_x10=%ld YC=%ld ESTOP=%d\r\n",
                      AppImu_GetGyroHoldEnable(),
                      AppImu_GetHeadingHoldActive(),
                      (long)f_to_i10(AppImu_GetYawKp()),
                      (long)f_to_i10(AppImu_GetYawKd()),
                      (long)AppImu_GetYawCorrLimit(),
                      g_bt_estop);

    BlueSerial_Printf("US EN=%d valid=%d dist_cm=%ld LIM=%d STOP=%d slow=%dcm block=%dcm\r\n",
                      AppSafety_GetUltrasonicEnable(),
                      AppSafety_GetUltraValid(),
                      (long)(AppSafety_GetUltraDistM() * 100.0f + 0.5f),
                      AppSafety_GetUltraLimited(),
                      AppSafety_GetUltraStop(),
                      (int)(APP_SAFETY_ULTRA_SLOW_DIST_M * 100.0f),
                      (int)(APP_SAFETY_ULTRA_STOP_DIST_M * 100.0f));

    BlueSerial_Printf("PI_TIMEOUT=%d SAFETY=0x%02X DBG=%d WDG_OK=%d WDG_RUN=%d\r\n",
                      AppSafety_GetPiTimeout(),
                      AppSafety_BuildStatusByte(g_bt_estop),
                      g_bt_debug_enable,
                      AppStatus_IsWatchdogFeedAllowed(),
                      AppStatus_IsWatchdogStarted());
}

void AppDebug_PrintStartup(void)
{
    BlueSerial_Printf("\r\nFLYT UART + MPU HOLD OFFICIAL READY\r\n");
    BlueSerial_Printf("MPU ID=0x%02X\r\n", AppImu_GetID());
    AppDebug_PrintHelp();
}

/*
 * 蓝牙命令解析原则：
 * 1. 长命令优先，例如 FL/FR/LIM/STEP 必须在 F/L 单字母运动命令前解析。
 * 2. 单字母运动命令 F/B/L/R 放在最后，避免截走参数命令。
 * 3. 运动命令成功后返回 1，让 main 刷新命令看门狗。
 */
uint8_t AppDebug_ProcessRx(void)
{
    char *cmd;
    int32_t v;
    uint8_t reset_watchdog = 0;

    if (!BlueSerial_RxFlag)
    {
        return 0;
    }

    cmd = BlueSerial_RxPacket;

    if (strcmp(cmd, "H") == 0 || strcmp(cmd, "help") == 0)
    {
        AppDebug_PrintHelp();
    }
    else if (strcmp(cmd, "STAT") == 0)
    {
        AppDebug_PrintStatus();
    }
    else if (strcmp(cmd, "ENCW") == 0)
{
    uint32_t i;
    uint16_t t3_start, t3_end;
    uint16_t t4_start, t4_end;

    uint8_t last_pa6, last_pa7, last_pb6, last_pb7;
    uint32_t edge_pa6 = 0, edge_pa7 = 0, edge_pb6 = 0, edge_pb7 = 0;

    uint8_t pa6, pa7, pb6, pb7;

    t3_start = TIM_GetCounter(TIM3);
    t4_start = TIM_GetCounter(TIM4);

    last_pa6 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
    last_pa7 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
    last_pb6 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
    last_pb7 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);

    BlueSerial_Printf("ENCW start: rotate wheels now...\r\n");

    for (i = 0; i < 2000; i++)
    {
        pa6 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
        pa7 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
        pb6 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
        pb7 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);

        if (pa6 != last_pa6) { edge_pa6++; last_pa6 = pa6; }
        if (pa7 != last_pa7) { edge_pa7++; last_pa7 = pa7; }
        if (pb6 != last_pb6) { edge_pb6++; last_pb6 = pb6; }
        if (pb7 != last_pb7) { edge_pb7++; last_pb7 = pb7; }

        Delay_ms(1);
    }

    t3_end = TIM_GetCounter(TIM3);
    t4_end = TIM_GetCounter(TIM4);

    BlueSerial_Printf("ENCW T3:%u->%u d=%d T4:%u->%u d=%d\r\n",
                      t3_start, t3_end, (int16_t)(t3_end - t3_start),
                      t4_start, t4_end, (int16_t)(t4_end - t4_start));

    BlueSerial_Printf("EDGE PA6=%lu PA7=%lu PB6=%lu PB7=%lu level=%d%d%d%d\r\n",
                      edge_pa6, edge_pa7, edge_pb6, edge_pb7,
                      GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6),
                      GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7),
                      GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6),
                      GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7));
}
    else if (strcmp(cmd, "ENC") == 0)
    {
        BlueSerial_Printf("ENC T3=%u T4=%u PA6=%d PA7=%d PB6=%d PB7=%d\r\n",
                          (unsigned int)TIM_GetCounter(TIM3),
                          (unsigned int)TIM_GetCounter(TIM4),
                          GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6),
                          GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7),
                          GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6),
                          GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7));
    }
    else if (strcmp(cmd, "TASK") == 0)
    {
        AppHealth_Print();
    }
    else if (strcmp(cmd, "S") == 0 || strcmp(cmd, "STOP") == 0)
    {
        AppChassis_Stop();
        BlueSerial_Printf("STOP\r\n");
    }
    else if (strcmp(cmd, "E1") == 0)
    {
        g_bt_estop = 1;
        AppChassis_Stop();
        BlueSerial_Printf("ESTOP=1\r\n");
    }
    else if (strcmp(cmd, "E0") == 0)
    {
        g_bt_estop = 0;
        AppChassis_ResetPid();
        BlueSerial_Printf("ESTOP=0\r\n");
    }
    else if (strcmp(cmd, "US1") == 0)
    {
        AppSafety_SetUltrasonicEnable(1);
        BlueSerial_Printf("US=1\r\n");
    }
    else if (strcmp(cmd, "US0") == 0)
    {
        AppSafety_SetUltrasonicEnable(0);
        BlueSerial_Printf("US=0\r\n");
    }
    else if (strcmp(cmd, "DBG1") == 0)
    {
        g_bt_debug_enable = 1;
        BlueSerial_Printf("DBG=1\r\n");
    }
    else if (strcmp(cmd, "DBG0") == 0)
    {
        g_bt_debug_enable = 0;
        BlueSerial_Printf("DBG=0\r\n");
    }
    else if (strcmp(cmd, "CAL") == 0)
    {
        AppChassis_Stop();
        AppImu_CalibrateGyroZ();
    }
    else if (strcmp(cmd, "YAW0") == 0)
    {
        AppImu_ResetYaw();
        BlueSerial_Printf("YAW=0\r\n");
    }
    else if (strcmp(cmd, "GYRO1") == 0)
    {
        AppImu_SetGyroHoldEnable(1);
        BlueSerial_Printf("GYRO=1\r\n");
    }
    else if (strcmp(cmd, "GYRO0") == 0)
    {
        AppImu_SetGyroHoldEnable(0);
        BlueSerial_Printf("GYRO=0\r\n");
    }
    else if (strncmp(cmd, "YKP", 3) == 0)
    {
        v = ReadNumberAfter(cmd, 3);
        if (v < 0)
            v = 0;
        if (v > 200)
            v = 200;
        AppImu_SetYawKp((float)v / 10.0f);
        BlueSerial_Printf("YKP=%ld/10\r\n", (long)v);
    }
    else if (strncmp(cmd, "YKD", 3) == 0)
    {
        v = ReadNumberAfter(cmd, 3);
        if (v < 0)
            v = 0;
        if (v > 100)
            v = 100;
        AppImu_SetYawKd((float)v / 10.0f);
        BlueSerial_Printf("YKD=%ld/10\r\n", (long)v);
    }
    else if (strncmp(cmd, "YC", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 200)
            v = 200;
        AppImu_SetYawCorrLimit(v);
        BlueSerial_Printf("YC=%ld\r\n", (long)v);
    }

    /* 轮速 PID / 补偿：长命令必须优先于 F/L/R/P/I 单字母命令 */
    else if (strncmp(cmd, "PL", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 80)
            v = 80;
        AppChassis_SetKpLeft((float)v / 100.0f);
        BlueSerial_Printf("PL=%ld/100\r\n", (long)v);
    }
    else if (strncmp(cmd, "PR", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 80)
            v = 80;
        AppChassis_SetKpRight((float)v / 100.0f);
        BlueSerial_Printf("PR=%ld/100\r\n", (long)v);
    }
    else if (strncmp(cmd, "IL", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 50)
            v = 50;
        AppChassis_SetKiLeft((float)v / 100.0f);
        BlueSerial_Printf("IL=%ld/100\r\n", (long)v);
    }
    else if (strncmp(cmd, "IR", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 50)
            v = 50;
        AppChassis_SetKiRight((float)v / 100.0f);
        BlueSerial_Printf("IR=%ld/100\r\n", (long)v);
    }
    else if (strncmp(cmd, "FL", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 200)
            v = 200;
        AppChassis_SetFFLeft((float)v / 1000.0f);
        BlueSerial_Printf("FL=%ld/1000\r\n", (long)v);
    }
    else if (strncmp(cmd, "FR", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 200)
            v = 200;
        AppChassis_SetFFRight((float)v / 1000.0f);
        BlueSerial_Printf("FR=%ld/1000\r\n", (long)v);
    }
    else if (strncmp(cmd, "SL", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 45)
            v = 45;
        AppChassis_SetStartLeft((int16_t)v);
        BlueSerial_Printf("SL=%ld\r\n", (long)v);
    }
    else if (strncmp(cmd, "SR", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 45)
            v = 45;
        AppChassis_SetStartRight((int16_t)v);
        BlueSerial_Printf("SR=%ld\r\n", (long)v);
    }
    else if (strncmp(cmd, "MS", 2) == 0)
    {
        v = ReadNumberAfter(cmd, 2);
        if (v < 0)
            v = 0;
        if (v > 250)
            v = 250;
        AppChassis_SetMinControlSpeed(v);
        BlueSerial_Printf("MS=%ld\r\n", (long)v);
    }
    else if (strncmp(cmd, "LIM", 3) == 0)
    {
        v = ReadNumberAfter(cmd, 3);
        if (v < 100)
            v = 100;
        if (v > 600)
            v = 600;
        AppChassis_SetTargetLimit(v);
        BlueSerial_Printf("LIM=%ld\r\n", (long)v);
    }
    else if (strncmp(cmd, "OUT", 3) == 0)
    {
        v = ReadNumberAfter(cmd, 3);
        if (v < 20)
            v = 20;
        if (v > 100)
            v = 100;
        AppChassis_SetPwmLimit((int16_t)v);
        BlueSerial_Printf("OUT=%ld\r\n", (long)v);
    }
    else if (strncmp(cmd, "STEP", 4) == 0)
    {
        v = ReadNumberAfter(cmd, 4);
        if (v < 1)
            v = 1;
        if (v > 50)
            v = 50;
        AppChassis_SetStep((int16_t)v);
        BlueSerial_Printf("STEP=%ld\r\n", (long)v);
    }
    else if (cmd[0] == 'P')
    {
        v = ReadNumberAfter(cmd, 1);
        if (v < 0)
            v = 0;
        if (v > 80)
            v = 80;
        AppChassis_SetKpBoth((float)v / 100.0f);
        BlueSerial_Printf("P=%ld/100\r\n", (long)v);
    }
    else if (cmd[0] == 'I')
    {
        v = ReadNumberAfter(cmd, 1);
        if (v < 0)
            v = 0;
        if (v > 50)
            v = 50;
        AppChassis_SetKiBoth((float)v / 100.0f);
        BlueSerial_Printf("I=%ld/100\r\n", (long)v);
    }

    /* 单字母运动命令放最后，避免截走 FL/FR/LIM 等参数命令 */
    else if (cmd[0] == 'F')
    {
        v = ReadNumberAfter(cmd, 1);
        if (v < 0)
            v = -v;
        if (v > AppChassis_GetTargetLimit())
            v = AppChassis_GetTargetLimit();
        reset_watchdog = SetBluetoothMotionTarget(v, v);
    }
    else if (cmd[0] == 'B')
    {
        v = ReadNumberAfter(cmd, 1);
        if (v < 0)
            v = -v;
        if (v > AppChassis_GetTargetLimit())
            v = AppChassis_GetTargetLimit();
        reset_watchdog = SetBluetoothMotionTarget(-v, -v);
    }
    else if (cmd[0] == 'L')
    {
        v = ReadNumberAfter(cmd, 1);
        if (v < 0)
            v = -v;
        if (v > AppChassis_GetTargetLimit())
            v = AppChassis_GetTargetLimit();
        reset_watchdog = SetBluetoothMotionTarget(-v, v);
    }
    else if (cmd[0] == 'R')
    {
        v = ReadNumberAfter(cmd, 1);
        if (v < 0)
            v = -v;
        if (v > AppChassis_GetTargetLimit())
            v = AppChassis_GetTargetLimit();
        reset_watchdog = SetBluetoothMotionTarget(v, -v);
    }
    else
    {
        BlueSerial_Printf("Unknown:%s\r\n", cmd);
        AppDebug_PrintHelp();
    }

    BlueSerial_RxFlag = 0;
    return reset_watchdog;
}

void AppDebug_ReportStep(void)
{
    if (!g_bt_debug_enable)
    {
        return;
    }

    g_bt_report_ms += g_ctrl_period_ms;

    if (g_bt_report_ms >= g_report_period_ms)
    {
        g_bt_report_ms = 0;

        BlueSerial_Printf("B=%ld,%ld T=%ld,%ld A=%ld,%ld P=%d,%d d=%d,%d yaw_x10=%ld gz_x100=%ld C=%ld H=%d US=%ld UV=%d UL=%d USTOP=%d total=%ld,%ld\r\n",
                          (long)AppChassis_GetBaseLeftMmps(),
                          (long)AppChassis_GetBaseRightMmps(),
                          (long)AppChassis_GetCtrlLeftMmps(),
                          (long)AppChassis_GetCtrlRightMmps(),
                          (long)AppChassis_GetLastActualLeftMmps(),
                          (long)AppChassis_GetLastActualRightMmps(),
                          AppChassis_GetLastPwmLeft(),
                          AppChassis_GetLastPwmRight(),
                          AppChassis_GetLastDeltaLeft(),
                          AppChassis_GetLastDeltaRight(),
                          (long)f_to_i10(AppImu_GetYawDeg()),
                          (long)f_to_i100(AppImu_GetGyroZDps()),
                          (long)AppImu_GetLastYawCorr(),
                          AppImu_GetHeadingHoldActive(),
                          (long)(AppSafety_GetUltraDistM() * 100.0f + 0.5f),
                          AppSafety_GetUltraValid(),
                          AppSafety_GetUltraLimited(),
                          AppSafety_GetUltraStop(),
                          (long)AppChassis_GetLeftTotalTicks(),
                          (long)AppChassis_GetRightTotalTicks());
    }
}
