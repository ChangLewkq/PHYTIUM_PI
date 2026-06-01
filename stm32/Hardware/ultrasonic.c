#include "stm32f10x.h"
#include "Delay.h"
#include "ultrasonic.h"

void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_Init(ECHO_PORT, &GPIO_InitStructure);
}

float Ultrasonic_Read(void)
{
    uint32_t timeout;
    uint32_t start;
    
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
    Delay_us(2);
    GPIO_SetBits(TRIG_PORT, TRIG_PIN);
    Delay_us(10);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
    
    timeout = 0;
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 0)
    {
        Delay_us(1);
        timeout++;
        if (timeout > 30000) return 2.0f;
    }
    
    start = 0;
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 1)
    {
        Delay_us(1);
        start++;
        if (start > 30000) return 2.0f;
    }
    
    return start * 0.00017f;
}