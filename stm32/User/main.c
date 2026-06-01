#include <rtthread.h>

#include "Delay.h"
#include "Motor.h"
#include "Encoder.h"
#include "FlytSerial.h"
#include "ultrasonic.h"

#include "app_chassis.h"
#include "app_debug.h"
#include "app_health.h"
#include "app_imu.h"
#include "app_safety.h"
#include "app_status.h"

/*
 * STM32 RT-Thread Full Vehicle Main V1.2
 *
 * 正式整车主程序：不做轻量化。V1.2 增加基于 app_health 的安全喂狗许可。
 *
 * 保留功能：
 * - 飞腾派 USART1 正式控制 /cmd_vel
 * - STM32 -> 飞腾派上行 ticks / gyro_z / safety_flags
 * - 蓝牙 USART2 调试 app_debug
 * - 编码器 / 轮速计算 / PID / PWM / 电机驱动
 * - 超声波安全限速/停车
 * - MPU6050 gyro_z / yaw / 航向保持
 * - PC13 LED 状态灯
 * - OLED 状态显示
 * - IWDG 看门狗预留，是否启用由 app_status.h 控制
 *
 * 控制优先级：
 * 1. 急停 / 超声波安全
 * 2. 飞腾派在线时，飞腾派 /cmd_vel 优先
 * 3. 飞腾派超时/离线时，蓝牙运动命令可短时接管
 *
 * 重要说明：
 * - 本文件是完整正式版，代码体积会明显大于 Lite 版。
 * - 如果 Keil 报 32KB 限制，请使用无限制 MDK、开启 MicroLIB/优化，
 *   或临时关闭 app_status.h 中的 OLED/IWDG。
 */

#define CTRL_PERIOD_MS              50
#define BT_REPORT_PERIOD_MS         500
#define CMD_WATCHDOG_MS             500
#define STATUS_PERIOD_MS            50

/*
 * 线程栈：
 * 如果出现 HardFault，优先适当增大 debug_thread_stack / status_thread_stack。
 */
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t init_thread_stack[1536];

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t debug_thread_stack[2048];

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t pi_thread_stack[1024];

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t motor_thread_stack[1536];

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t status_thread_stack[1536];

static struct rt_thread init_thread;
static struct rt_thread debug_thread;
static struct rt_thread pi_thread;
static struct rt_thread motor_thread;
static struct rt_thread status_thread;

static volatile rt_uint8_t g_system_ready = 0;
static volatile rt_uint8_t g_status_ready = 0;

static volatile rt_uint16_t g_pi_watchdog_ms = CMD_WATCHDOG_MS;
static volatile rt_uint16_t g_bt_watchdog_ms = CMD_WATCHDOG_MS;

/*
 * 系统初始化线程
 *
 * 说明：
 * - 所有硬件和 app 模块初始化集中放在这里，避免多个线程同时初始化外设。
 * - 初始化完成后置位 g_system_ready。
 */
static void init_thread_entry(void *parameter)
{
    Delay_Init();
    AppHealth_Init();
    AppHealth_Beat(APP_HEALTH_INIT);

    /*
     * USART 初始化：
     * - FlytSerial_Init: USART1 飞腾派通信
     * - AppDebug_Init: 内部调用 BlueSerial_Init，USART2 蓝牙调试
     */
    FlytSerial_Init();
    AppDebug_Init(CTRL_PERIOD_MS, BT_REPORT_PERIOD_MS, CMD_WATCHDOG_MS);

    /*
     * 底层硬件初始化
     */
    Motor_Init();
    Encoder_Init();
    Ultrasonic_Init();

    /*
     * 应用模块初始化
     */
    AppSafety_Init();
    AppImu_Init(CTRL_PERIOD_MS);
    AppChassis_Init();

    /*
     * 状态显示模块：
     * - LED
     * - OLED
     * - IWDG 预留
     *
     * 是否使用 OLED/IWDG 由 app_status.h 控制：
     * APP_STATUS_USE_OLED
     * APP_STATUS_USE_IWDG
     */
    AppStatus_Init();
    g_status_ready = 1;

    /*
     * 初始安全状态
     */
    AppChassis_Stop();
    AppChassis_ClearDelta();
    AppSafety_SetPiTimeout(1);

    rt_thread_mdelay(300);

    AppDebug_PrintStartup();

    /*
     * MPU6050 零偏校准。上电后请保持小车静止。
     */
    AppImu_CalibrateGyroZ();

    AppDebug_PrintStatus();

    g_system_ready = 1;

    /*
     * init 线程完成后保持低频休眠，不退出也可以。
     */
    while (1)
    {
        AppHealth_Beat(APP_HEALTH_INIT);
        rt_thread_mdelay(1000);
    }
}

/*
 * 蓝牙调试线程
 *
 * 功能：
 * - [H]
 * - [STAT]
 * - [DBG1]/[DBG0]
 * - [P]/[I]/[FL]/[FR]/[LIM]/[OUT]/[STEP]
 * - [GYRO1]/[GYRO0]/[YAW0]/[CAL]
 * - [US1]/[US0]
 * - [F]/[B]/[L]/[R]/[S]/[E1]/[E0]
 *
 * 蓝牙运动命令只有在飞腾派超时时才会作为运动控制入口生效；
 * 飞腾派在线时，pi_thread 会持续用 /cmd_vel 目标覆盖底盘目标。
 */
static void debug_thread_entry(void *parameter)
{
    while (!g_system_ready)
    {
        rt_thread_mdelay(10);
    }

    while (1)
    {
        if (AppDebug_ProcessRx())
        {
            /*
             * AppDebug_ProcessRx 返回 1 表示收到蓝牙运动命令并成功设置目标。
             */
            g_bt_watchdog_ms = 0;
        }

        AppDebug_ReportStep();
        AppHealth_Beat(APP_HEALTH_DEBUG);

        rt_thread_mdelay(CTRL_PERIOD_MS);
    }
}

/*
 * 飞腾派通信线程
 *
 * 功能：
 * - 读取 USART1 下行速度包
 * - 更新底盘基础目标速度
 * - 维护飞腾派心跳
 *
 * 注意：
 * - FlytSerial 的 USART1 接收状态机仍在中断里。
 * - 本线程只取最新完整包。
 */
static void pi_thread_entry(void *parameter)
{
    FlytCmd_t cmd;

    while (!g_system_ready)
    {
        rt_thread_mdelay(10);
    }

    while (1)
    {
        if (FlytSerial_GetLatestCmd(&cmd))
        {
            AppChassis_UpdateTargetFromCmd(cmd.linear_mps,
                                           cmd.angular_radps,
                                           AppDebug_GetEstop());

            g_pi_watchdog_ms = 0;
            AppSafety_SetPiTimeout(0);
        }

        /*
         * USART1 下行约 20Hz，10ms 检查足够。
         */
        AppHealth_Beat(APP_HEALTH_PI);
        rt_thread_mdelay(10);
    }
}

/*
 * 电机控制线程
 *
 * 周期：50ms，与当前裸机稳定版保持一致。
 *
 * 功能：
 * - 飞腾派心跳判断
 * - 蓝牙备用控制超时判断
 * - 超声波安全更新
 * - MPU 更新
 * - 航向保持修正
 * - 编码器读取
 * - 轮速计算
 * - PID 更新
 * - PWM 输出
 * - STM32 上行状态帧发送
 */
static void motor_thread_entry(void *parameter)
{
    while (!g_system_ready)
    {
        rt_thread_mdelay(10);
    }

    while (1)
    {
        /*
         * 飞腾派心跳。
         * 飞腾派在线：由 pi_thread 持续更新目标速度。
         * 飞腾派超时：设置 PI_TIMEOUT，并允许蓝牙短时接管。
         */
        if (g_pi_watchdog_ms >= CMD_WATCHDOG_MS)
        {
            AppSafety_SetPiTimeout(1);
        }
        else
        {
            g_pi_watchdog_ms += CTRL_PERIOD_MS;
            AppSafety_SetPiTimeout(0);
        }

        /*
         * 蓝牙运动命令超时。
         * 蓝牙只作为飞腾派离线时的备用调试控制。
         */
        if (g_bt_watchdog_ms < CMD_WATCHDOG_MS)
        {
            g_bt_watchdog_ms += CTRL_PERIOD_MS;
        }

        if (AppSafety_GetPiTimeout())
        {
            /*
             * 飞腾派离线：
             * - 如果蓝牙也超时，则停车。
             * - 如果蓝牙刚发过运动命令，则允许短时运动。
             */
            if (g_bt_watchdog_ms >= CMD_WATCHDOG_MS)
            {
                AppChassis_Stop();
            }
        }
        else
        {
            /*
             * 飞腾派在线：
             * 不在这里强制 Stop。
             * 飞腾派 / uart_bridge 应该持续发零速度作为待机心跳。
             */
        }

        AppSafety_Update(CTRL_PERIOD_MS);

        /*
         * 真实底盘闭环与上行。
         */
        AppChassis_ControlAndUplinkStep(AppDebug_GetEstop());
        AppHealth_Beat(APP_HEALTH_MOTOR);

        rt_thread_mdelay(CTRL_PERIOD_MS);
    }
}

/*
 * 状态显示线程
 *
 * 功能：
 * - PC13 LED 状态
 * - OLED 状态
 * - IWDG 喂狗预留
 */
static void status_thread_entry(void *parameter)
{
    while (!g_status_ready)
    {
        rt_thread_mdelay(10);
    }

    while (1)
    {
        AppStatus_Step(STATUS_PERIOD_MS);
        AppHealth_Beat(APP_HEALTH_STATUS);
        rt_thread_mdelay(STATUS_PERIOD_MS);
    }
}

int main(void)
{
    /*
     * 优先级数值越小，优先级越高。
     *
     * motor: 最高，保证闭环周期
     * pi:    高，及时处理飞腾派速度包
     * debug: 中，蓝牙调试
     * status:低，OLED/LED/看门狗
     * init:  中高，初始化完成后休眠
     */

    rt_thread_init(&init_thread,
                   "init",
                   init_thread_entry,
                   RT_NULL,
                   init_thread_stack,
                   sizeof(init_thread_stack),
                   6,
                   10);
    rt_thread_startup(&init_thread);

    rt_thread_init(&motor_thread,
                   "motor",
                   motor_thread_entry,
                   RT_NULL,
                   motor_thread_stack,
                   sizeof(motor_thread_stack),
                   8,
                   10);
    rt_thread_startup(&motor_thread);

    rt_thread_init(&pi_thread,
                   "pi",
                   pi_thread_entry,
                   RT_NULL,
                   pi_thread_stack,
                   sizeof(pi_thread_stack),
                   12,
                   10);
    rt_thread_startup(&pi_thread);

    rt_thread_init(&debug_thread,
                   "debug",
                   debug_thread_entry,
                   RT_NULL,
                   debug_thread_stack,
                   sizeof(debug_thread_stack),
                   18,
                   10);
    rt_thread_startup(&debug_thread);

    rt_thread_init(&status_thread,
                   "status",
                   status_thread_entry,
                   RT_NULL,
                   status_thread_stack,
                   sizeof(status_thread_stack),
                   24,
                   10);
    rt_thread_startup(&status_thread);

    return 0;
}
