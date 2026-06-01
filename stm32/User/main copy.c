#include <rtthread.h>
#include <stdint.h>

#include "stm32f10x.h"

#include "Motor.h"
#include "Encoder.h"
#include "ChassisMap.h"
#include "BlueSerial.h"

/*
 * simple_motor_test_main.c
 *
 * 临时开环测试：
 * 1. 左轮单独转 5 秒
 * 2. 右轮单独转 5 秒
 * 3. 左右轮同时转 5 秒
 *
 * 不使用 PID / IMU / 超声波 / OLED / IWDG。
 */

#define TEST_PWM        60     /* 测试 PWM，太小不转就改成 45 或 50 */
#define RUN_TIME_MS     5000
#define STOP_TIME_MS    2000

static void Test_Stop(void)
{
    ChassisMap_SetWheelPWM(0, 0);
}

static void Test_LeftWheel(void)
{
    BlueSerial_Printf("\r\nTEST 1: LEFT wheel forward 5s, PWM=%d\r\n", TEST_PWM);
    ChassisMap_SetWheelPWM(TEST_PWM, 0);
    rt_thread_mdelay(RUN_TIME_MS);
    Test_Stop();
    BlueSerial_Printf("LEFT wheel test done\r\n");
}

static void Test_RightWheel(void)
{
    BlueSerial_Printf("\r\nTEST 2: RIGHT wheel forward 5s, PWM=%d\r\n", TEST_PWM);
    ChassisMap_SetWheelPWM(0, TEST_PWM);
    rt_thread_mdelay(RUN_TIME_MS);
    Test_Stop();
    BlueSerial_Printf("RIGHT wheel test done\r\n");
}

static void Test_BothWheel(void)
{
    BlueSerial_Printf("\r\nTEST 3: BOTH wheels forward 5s, PWM=%d\r\n", TEST_PWM);
    ChassisMap_SetWheelPWM(TEST_PWM, TEST_PWM);
    rt_thread_mdelay(RUN_TIME_MS);
    Test_Stop();
    BlueSerial_Printf("BOTH wheels test done\r\n");
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    BlueSerial_Init();
    Motor_Init();
    Encoder_Init();

    Test_Stop();

    rt_thread_mdelay(1000);

    BlueSerial_Printf("\r\n===== SIMPLE MOTOR OPEN LOOP TEST START =====\r\n");
    BlueSerial_Printf("PWM=%d, each test runs 5 seconds\r\n", TEST_PWM);

    Test_LeftWheel();

    rt_thread_mdelay(STOP_TIME_MS);

    Test_RightWheel();

    rt_thread_mdelay(STOP_TIME_MS);

    Test_BothWheel();

    Test_Stop();

    BlueSerial_Printf("\r\n===== SIMPLE MOTOR OPEN LOOP TEST FINISHED =====\r\n");
    BlueSerial_Printf("Motor stopped. Reset board to test again.\r\n");

    while (1)
    {
        Test_Stop();
        rt_thread_mdelay(1000);
    }
}