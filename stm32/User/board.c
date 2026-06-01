#include <rtthread.h>
#include <rthw.h>
#include "stm32f10x.h"

void rt_hw_board_init(void)
{
    SystemCoreClockUpdate();

    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /*
     * SysTick 作为 RT-Thread 系统节拍。
     * 优先级设低，避免影响 USART、编码器等中断。
     */
    NVIC_SetPriority(SysTick_IRQn, 0x0F);
}