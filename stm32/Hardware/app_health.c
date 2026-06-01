#include "app_health.h"

#include <rtthread.h>
#include "BlueSerial.h"

#define APP_HEALTH_MOTOR_MAX_AGE_MS     200UL
#define APP_HEALTH_PI_MAX_AGE_MS        200UL
#define APP_HEALTH_DEBUG_MAX_AGE_MS     500UL
#define APP_HEALTH_STATUS_MAX_AGE_MS    500UL

static volatile uint32_t g_health_count[APP_HEALTH_MAX];
static volatile uint32_t g_health_last_tick[APP_HEALTH_MAX];

static const char *g_health_name[APP_HEALTH_MAX] =
{
    "init",
    "motor",
    "pi",
    "debug",
    "status"
};

void AppHealth_Init(void)
{
    uint8_t i;

    for (i = 0; i < APP_HEALTH_MAX; i++)
    {
        g_health_count[i] = 0;
        g_health_last_tick[i] = 0;
    }
}

void AppHealth_Beat(AppHealthId_t id)
{
    if (id >= APP_HEALTH_MAX)
    {
        return;
    }

    g_health_count[id]++;
    g_health_last_tick[id] = rt_tick_get();
}

uint32_t AppHealth_GetCount(AppHealthId_t id)
{
    if (id >= APP_HEALTH_MAX)
    {
        return 0;
    }

    return g_health_count[id];
}

uint32_t AppHealth_GetLastTick(AppHealthId_t id)
{
    if (id >= APP_HEALTH_MAX)
    {
        return 0;
    }

    return g_health_last_tick[id];
}

uint32_t AppHealth_GetAgeMs(AppHealthId_t id)
{
    uint32_t now;
    uint32_t last;
    uint32_t diff_ticks;

    if (id >= APP_HEALTH_MAX)
    {
        return 0xFFFFFFFFUL;
    }

    now = rt_tick_get();
    last = g_health_last_tick[id];

    if (last == 0)
    {
        return 0xFFFFFFFFUL;
    }

    diff_ticks = now - last;

    return (diff_ticks * 1000UL) / RT_TICK_PER_SECOND;
}

uint8_t AppHealth_IsAlive(AppHealthId_t id, uint32_t max_age_ms)
{
    uint32_t age;

    if (id >= APP_HEALTH_MAX)
    {
        return 0;
    }

    if (g_health_last_tick[id] == 0)
    {
        return 0;
    }

    age = AppHealth_GetAgeMs(id);

    if (age <= max_age_ms)
    {
        return 1;
    }

    return 0;
}

uint8_t AppHealth_AllCriticalAlive(void)
{
    if (!AppHealth_IsAlive(APP_HEALTH_MOTOR, APP_HEALTH_MOTOR_MAX_AGE_MS))
    {
        return 0;
    }

    if (!AppHealth_IsAlive(APP_HEALTH_PI, APP_HEALTH_PI_MAX_AGE_MS))
    {
        return 0;
    }

    if (!AppHealth_IsAlive(APP_HEALTH_DEBUG, APP_HEALTH_DEBUG_MAX_AGE_MS))
    {
        return 0;
    }

    if (!AppHealth_IsAlive(APP_HEALTH_STATUS, APP_HEALTH_STATUS_MAX_AGE_MS))
    {
        return 0;
    }

    return 1;
}

void AppHealth_Print(void)
{
    uint8_t i;

    BlueSerial_Printf("\r\nTASK HEALTH tick=%lu all_ok=%d\r\n",
                      (unsigned long)rt_tick_get(),
                      AppHealth_AllCriticalAlive());

    for (i = 0; i < APP_HEALTH_MAX; i++)
    {
        BlueSerial_Printf("%s cnt=%lu last=%lu age_ms=%lu\r\n",
                          g_health_name[i],
                          (unsigned long)g_health_count[i],
                          (unsigned long)g_health_last_tick[i],
                          (unsigned long)AppHealth_GetAgeMs((AppHealthId_t)i));
    }
}
