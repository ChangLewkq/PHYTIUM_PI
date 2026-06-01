#include "stm32f10x.h"
#include "BlueSerial.h"
#include <stdio.h>
#include <stdarg.h>

char BlueSerial_RxPacket[BLUESERIAL_RX_PACKET_SIZE];
uint8_t BlueSerial_RxFlag = 0;

static uint8_t RxIndex = 0;
static uint8_t BracketMode = 0;

void BlueSerial_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void BlueSerial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        BlueSerial_SendByte(String[i]);
    }
}

void BlueSerial_Printf(char *format, ...)
{
    char String[160];
    va_list arg;

    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);

    BlueSerial_SendString(String);
}

void BlueSerial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 USART2_TX */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 USART2_RX */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

/*
 * 支持两种命令格式：
 *
 * 1. 旧格式：[F180]
 *    收到 '[' 开始，收到 ']' 结束。
 *
 * 2. 新格式：F180 + 回车/换行
 *    手机串口助手需要打开“发送新行/追加CRLF”。
 *
 * 注意：
 * 不加 [] 时，必须有 \r 或 \n 作为结束符。
 */
void USART2_IRQHandler(void)
{
    uint8_t RxData;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        RxData = USART_ReceiveData(USART2);

        if (RxData == '[')
        {
            BracketMode = 1;
            RxIndex = 0;
        }
        else if (BracketMode)
        {
            if (RxData == ']')
            {
                BlueSerial_RxPacket[RxIndex] = '\0';
                BlueSerial_RxFlag = 1;
                BracketMode = 0;
                RxIndex = 0;
            }
            else
            {
                if (RxIndex < BLUESERIAL_RX_PACKET_SIZE - 1)
                {
                    BlueSerial_RxPacket[RxIndex++] = RxData;
                }
                else
                {
                    RxIndex = 0;
                    BracketMode = 0;
                }
            }
        }
        else
        {
            if (RxData == '\r' || RxData == '\n')
            {
                if (RxIndex > 0)
                {
                    BlueSerial_RxPacket[RxIndex] = '\0';
                    BlueSerial_RxFlag = 1;
                    RxIndex = 0;
                }
            }
            else
            {
                if (RxIndex < BLUESERIAL_RX_PACKET_SIZE - 1)
                {
                    BlueSerial_RxPacket[RxIndex++] = RxData;
                }
                else
                {
                    RxIndex = 0;
                }
            }
        }

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
