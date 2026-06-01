#include "Delay.h"
#include "stm32f10x.h"
#include <stdint.h>
#include <rtthread.h>

/*
 * RT-Thread compatible Delay.c for STM32F103 / Cortex-M3.
 *
 * Important:
 * - SysTick is owned by RT-Thread.
 * - Delay_ms() uses rt_thread_mdelay().
 * - Delay_us() uses DWT CYCCNT by direct register address.
 * - This version does NOT depend on CMSIS exposing DWT structure.
 */

/* Cortex-M3 CoreDebug DEMCR register */
#define DBG_DEMCR              (*(volatile uint32_t *)0xE000EDFCUL)
#define DBG_DEMCR_TRCENA       (1UL << 24)

/* Cortex-M3 DWT registers */
#define DWT_CTRL_REG           (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT_REG         (*(volatile uint32_t *)0xE0001004UL)
#define DWT_CTRL_CYCCNTENA     (1UL << 0)

static uint8_t g_delay_dwt_ready = 0;

void Delay_Init(void)
{
    if (g_delay_dwt_ready)
    {
        return;
    }

    /*
     * Enable DWT cycle counter.
     * STM32F103 Cortex-M3 supports DWT CYCCNT.
     */
    DBG_DEMCR |= DBG_DEMCR_TRCENA;
    DWT_CYCCNT_REG = 0;
    DWT_CTRL_REG |= DWT_CTRL_CYCCNTENA;

    g_delay_dwt_ready = 1;
}

void Delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (us == 0)
    {
        return;
    }

    if (!g_delay_dwt_ready)
    {
        Delay_Init();
    }

    /*
     * SystemCoreClock should be 72000000 in your STM32F103 project.
     * The calculation still follows SystemCoreClock if it changes later.
     */
    ticks = (SystemCoreClock / 1000000UL) * us;
    start = DWT_CYCCNT_REG;

    while ((uint32_t)(DWT_CYCCNT_REG - start) < ticks)
    {
        /* busy wait, no SysTick access */
    }
}

void Delay_ms(uint32_t ms)
{
    if (ms == 0)
    {
        return;
    }

    /*
     * Must be called after RT-Thread scheduler has started,
     * such as inside user main thread or any RT-Thread thread.
     */
    rt_thread_mdelay(ms);
}

void Delay_s(uint32_t s)
{
    while (s--)
    {
        Delay_ms(1000);
    }
}
