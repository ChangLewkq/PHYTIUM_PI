#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "config.h"
/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB的时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB12、PB13、PB14和PB15引脚初始化为推挽输出
	
	PWM_Init();												//初始化直流电机的底层PWM
}

/**
  * 函    数：直流电机设置PWM
  * 参    数：n 指定左电机还是右电机，范围：1（左电机），2（右电机）
  * 参    数：PWM 要设置的PWM值，范围：-100~100（负数为反转）
  * 返 回 值：无
  */
void Motor_SetPWM(uint8_t n, int8_t PWM)
{
	if (n == 1)			//指定左电机
	{
		if (PWM >= 0)							//如果设置正转的PWM
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_12);	//PB12置高电平
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);	//PB13置低电平
			PWM_SetCompare1(PWM);				//设置PWM占空比
		}
		else									//否则，即设置反转的PWM
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);	//PB12置低电平
			GPIO_SetBits(GPIOB, GPIO_Pin_13);	//PB13置高电平
			PWM_SetCompare1(-PWM);				//设置PWM占空比
		}
	}
	else if (n == 2)	//指定右电机
	{
		if (PWM >= 0)							//如果设置正转的PWM
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_14);	//PB14置低电平
			GPIO_SetBits(GPIOB, GPIO_Pin_15);	//PB15置高电平
			PWM_SetCompare2(PWM);				//设置PWM占空比
		}
		else									//否则，即设置反转的PWM
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_14);	//PB14置高电平
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);	//PB15置低电平
			PWM_SetCompare2(-PWM);				//设置PWM占空比
		}
	}
}

/**
  * 函    数：根据速度(m/s)设置电机PWM
  * 参    数：n 指定左电机还是右电机，范围：1（左电机），2（右电机）
  * 参    数：Speed 速度，单位：m/s，正转前进，负转后退
  * 返 回 值：无
  */
void Motor_SetSpeed(uint8_t n, float Speed)
{
    int16_t PWM;
    
    /* 线速度 → 轮速(rps) → PWM */
    /* 轮速 = 线速度 / (π * 轮径) */
    float WheelRPS = Speed / (3.14159f * WHEEL_DIAMETER);
    
    /* 轮速(rps) → PWM 线性映射 */
    PWM = (int16_t)(WheelRPS / MAX_WHEEL_RPS * PWM_MAX);
    
    /* PWM限幅 */
    if (PWM > PWM_MAX)  PWM = PWM_MAX;
    if (PWM < PWM_MIN)  PWM = PWM_MIN;
    
    Motor_SetPWM(n, (int8_t)PWM);
}

/**
  * 函    数：电机急停
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Stop(void)
{
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
}
