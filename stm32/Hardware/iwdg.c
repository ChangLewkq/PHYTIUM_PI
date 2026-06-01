#include "stm32f10x.h"
#include "iwdg.h"

/**
  * 函    数：独立看门狗初始化
  * 参    数：Prescaler 预分频值，范围：IWDG_Prescaler_4 ~ IWDG_Prescaler_256
  * 参    数：Reload 重装载值，范围：0x000 ~ 0xFFF（最大4095）
  * 返 回 值：无
  * 
  * 说    明：超时时间计算：
  *           Tout = (4 * 2^Prescaler) / 40KHz * Reload
  *           常用配置：
  *           预分频64，重装载625  → 1秒超时
  *           预分频64，重装载1000 → 1.6秒超时
  *           预分频128，重装载1250 → 4秒超时
  */
void IWDG_Init(uint16_t Prescaler, uint16_t Reload)
{
    /* 使能对IWDG_PR和IWDG_RLR寄存器的写访问 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    
    /* 设置预分频值 */
    IWDG_SetPrescaler(Prescaler);
    
    /* 设置重装载值 */
    IWDG_SetReload(Reload);
    
    /* 重装载计数器（喂狗） */
    IWDG_ReloadCounter();
    
    /* 使能独立看门狗 */
    IWDG_Enable();
}

/**
  * 函    数：喂狗
  * 参    数：无
  * 返 回 值：无
  */
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}