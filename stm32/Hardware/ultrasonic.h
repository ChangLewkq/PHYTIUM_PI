#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

#define TRIG_PORT   GPIOA
#define TRIG_PIN    GPIO_Pin_4
#define ECHO_PORT   GPIOA
#define ECHO_PIN    GPIO_Pin_5

void Ultrasonic_Init(void);
float Ultrasonic_Read(void);

#endif