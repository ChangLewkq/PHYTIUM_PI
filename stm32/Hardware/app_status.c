#include "app_status.h"

#include "LED.h"
#include "app_chassis.h"
#include "app_debug.h"
#include "app_health.h"
#include "app_imu.h"
#include "app_safety.h"

#if APP_STATUS_USE_OLED
#include "OLED.h"
#endif

#if APP_STATUS_USE_IWDG
#include "iwdg.h"
#endif

#include <stdint.h>

static uint16_t g_led_timer_ms = 0;
static uint16_t g_oled_timer_ms = 0;
static uint16_t g_wdg_timer_ms = 0;

static uint8_t g_iwdg_started = 0;
static uint16_t g_iwdg_ready_ms = 0;

static int32_t abs_i32_local(int32_t x)
{
    return (x >= 0) ? x : -x;
}

static uint8_t chassis_is_moving(void)
{
    if (abs_i32_local(AppChassis_GetCtrlLeftMmps()) > 5)
    {
        return 1;
    }

    if (abs_i32_local(AppChassis_GetCtrlRightMmps()) > 5)
    {
        return 1;
    }

    if (abs_i32_local(AppChassis_GetLastActualLeftMmps()) > 5)
    {
        return 1;
    }

    if (abs_i32_local(AppChassis_GetLastActualRightMmps()) > 5)
    {
        return 1;
    }

    return 0;
}

static uint16_t get_led_period_ms(void)
{
    if (AppSafety_GetPiTimeout())
    {
        return APP_STATUS_LED_TIMEOUT_MS;
    }

    if (AppDebug_GetDebugEnable())
    {
        return APP_STATUS_LED_DEBUG_MS;
    }

    if (chassis_is_moving())
    {
        return APP_STATUS_LED_MOVE_MS;
    }

    return APP_STATUS_LED_NORMAL_MS;
}

static void AppStatus_LedStep(uint16_t dt_ms)
{
    if (AppDebug_GetEstop() || AppSafety_GetUltraStop())
    {
        LED_ON();
        g_led_timer_ms = 0;
        return;
    }

    g_led_timer_ms += dt_ms;

    if (g_led_timer_ms >= get_led_period_ms())
    {
        g_led_timer_ms = 0;
        LED_Turn();
    }
}

uint8_t AppStatus_IsWatchdogFeedAllowed(void)
{
    return AppHealth_AllCriticalAlive();
}

uint8_t AppStatus_IsWatchdogStarted(void)
{
    return g_iwdg_started;
}

#if APP_STATUS_USE_OLED
static void AppStatus_OledStep(uint16_t dt_ms)
{
    uint8_t safety;
    const char *mode;

    g_oled_timer_ms += dt_ms;

    if (g_oled_timer_ms < APP_STATUS_OLED_PERIOD_MS)
    {
        return;
    }

    g_oled_timer_ms = 0;

    safety = AppSafety_BuildStatusByte(AppDebug_GetEstop());

    if (AppDebug_GetEstop())
    {
        mode = "ESTOP";
    }
    else if (AppSafety_GetPiTimeout())
    {
        mode = "TIMEOUT";
    }
    else if (AppSafety_GetUltraStop())
    {
        mode = "USTOP";
    }
    else if (AppDebug_GetDebugEnable())
    {
        mode = "DEBUG";
    }
    else if (chassis_is_moving())
    {
        mode = "MOVE";
    }
    else
    {
        mode = "IDLE";
    }

    OLED_Clear();

    OLED_Printf(0, 0, OLED_6X8, "MODE:%s S:%02X", mode, safety);
    OLED_Printf(0, 10, OLED_6X8, "T L%ld R%ld",
                (long)AppChassis_GetCtrlLeftMmps(),
                (long)AppChassis_GetCtrlRightMmps());

    OLED_Printf(0, 20, OLED_6X8, "A L%ld R%ld",
                (long)AppChassis_GetLastActualLeftMmps(),
                (long)AppChassis_GetLastActualRightMmps());

    OLED_Printf(0, 30, OLED_6X8, "US:%ldcm V%d L%d S%d",
                (long)(AppSafety_GetUltraDistM() * 100.0f + 0.5f),
                AppSafety_GetUltraValid(),
                AppSafety_GetUltraLimited(),
                AppSafety_GetUltraStop());

    OLED_Printf(0, 40, OLED_6X8, "Y:%ld G:%ld I:%d",
                (long)(AppImu_GetYawDeg() * 10.0f),
                (long)(AppImu_GetGyroZDps() * 100.0f),
                AppImu_IsOnline());

    OLED_Printf(0, 50, OLED_6X8, "PI:%d W:%d R:%d",
                AppSafety_GetPiTimeout(),
                AppStatus_IsWatchdogFeedAllowed(),
                AppStatus_IsWatchdogStarted());

    OLED_Update();
}
#endif

#if APP_STATUS_USE_IWDG
static void AppStatus_WatchdogStep(uint16_t dt_ms)
{
    g_wdg_timer_ms += dt_ms;

    if (g_wdg_timer_ms < APP_STATUS_WDG_PERIOD_MS)
    {
        return;
    }

    g_wdg_timer_ms = 0;

    /*
     * 延迟启动阶段：
     * IWDG 一旦启动就不能停止，所以必须等系统关键任务都稳定后再启动。
     */
    if (!g_iwdg_started)
    {
        if (AppStatus_IsWatchdogFeedAllowed())
        {
            if (g_iwdg_ready_ms < APP_STATUS_IWDG_START_DELAY_MS)
            {
                g_iwdg_ready_ms += APP_STATUS_WDG_PERIOD_MS;
            }

            if (g_iwdg_ready_ms >= APP_STATUS_IWDG_START_DELAY_MS)
            {
                /*
                 * 预分频64，重装载1000，约1.6s超时。
                 * 启动后立即喂一次。
                 */
                IWDG_Init(IWDG_Prescaler_64, 1000);
                IWDG_Feed();
                g_iwdg_started = 1;
            }
        }
        else
        {
            g_iwdg_ready_ms = 0;
        }

        return;
    }

    /*
     * IWDG 已启动：
     * 只有关键任务全部健康时才喂狗。
     * 任一关键线程卡死，则停止喂狗，等待硬件复位。
     */
    if (AppStatus_IsWatchdogFeedAllowed())
    {
        IWDG_Feed();
    }
}
#endif

void AppStatus_Init(void)
{
    g_led_timer_ms = 0;
    g_oled_timer_ms = 0;
    g_wdg_timer_ms = 0;
    g_iwdg_started = 0;
    g_iwdg_ready_ms = 0;

    LED_Init();
    LED_OFF();

#if APP_STATUS_USE_OLED
    OLED_Init();
    OLED_Clear();
    OLED_Printf(0, 0, OLED_6X8, "STM32 Chassis");
    OLED_Printf(0, 12, OLED_6X8, "Status Init...");
    OLED_Printf(0, 24, OLED_6X8, "IWDG delay start");
    OLED_Update();
#endif
}

void AppStatus_Step(uint16_t dt_ms)
{
    AppStatus_LedStep(dt_ms);

#if APP_STATUS_USE_OLED
    AppStatus_OledStep(dt_ms);
#endif

#if APP_STATUS_USE_IWDG
    AppStatus_WatchdogStep(dt_ms);
#endif
}
