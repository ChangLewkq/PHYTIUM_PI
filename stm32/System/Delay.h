#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * RT-Thread compatible delay interface.
 *
 * Delay_ms / Delay_s:
 *   Use RT-Thread delay.
 *
 * Delay_us:
 *   Use Cortex-M3 DWT CYCCNT direct registers.
 *   Does not touch SysTick.
 */

void Delay_Init(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif
