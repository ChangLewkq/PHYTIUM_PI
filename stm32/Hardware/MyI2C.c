#include "stm32f10x.h"
#include "Delay.h"
#include "MyI2C.h"

/*
 * MyI2C V1.3.1 for RT-Thread
 *
 * 修复：
 * - 增加函数原型/调整顺序，兼容 Keil ARMCC C89 编译规则。
 *
 * 引脚：
 * SCL = PB10
 * SDA = PB11
 */

#define MYI2C_SCL_PIN       GPIO_Pin_10
#define MYI2C_SDA_PIN       GPIO_Pin_11
#define MYI2C_GPIO_PORT     GPIOB

#define MYI2C_DELAY_US      5

static void MyI2C_Delay(void)
{
    Delay_us(MYI2C_DELAY_US);
}

void MyI2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(MYI2C_GPIO_PORT, MYI2C_SCL_PIN, (BitAction)(BitValue ? 1 : 0));
    MyI2C_Delay();
}

void MyI2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(MYI2C_GPIO_PORT, MYI2C_SDA_PIN, (BitAction)(BitValue ? 1 : 0));
    MyI2C_Delay();
}

uint8_t MyI2C_R_SDA(void)
{
    uint8_t BitValue;

    MyI2C_Delay();
    BitValue = GPIO_ReadInputDataBit(MYI2C_GPIO_PORT, MYI2C_SDA_PIN);
    MyI2C_Delay();

    return BitValue;
}

void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

/*
 * I2C 总线恢复：
 * 如果从机异常占用 SDA，主机发送 9 个 SCL 脉冲，让从机释放总线。
 */
void MyI2C_BusRecover(void)
{
    uint8_t i;

    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);

    for (i = 0; i < 9; i++)
    {
        MyI2C_W_SCL(0);
        MyI2C_W_SCL(1);
    }

    MyI2C_Stop();
}

void MyI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin = MYI2C_SCL_PIN | MYI2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MYI2C_GPIO_PORT, &GPIO_InitStructure);

    GPIO_SetBits(MYI2C_GPIO_PORT, MYI2C_SCL_PIN | MYI2C_SDA_PIN);

    MyI2C_BusRecover();
}

void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SDA(!!(Byte & (0x80 >> i)));
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t Byte = 0x00;

    MyI2C_W_SDA(1);

    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1);

        if (MyI2C_R_SDA())
        {
            Byte |= (0x80 >> i);
        }

        MyI2C_W_SCL(0);
    }

    return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;

    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);

    return AckBit;
}
