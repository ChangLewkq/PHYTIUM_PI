#include "FlytSerial.h"

static volatile FlytCmd_t g_latest_cmd;
static volatile uint8_t g_cmd_new_flag = 0;

void FlytSerial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 TX */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 RX */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void FlytSerial_SendBytes(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        USART_SendData(USART1, buf[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

void FlytSerial_SendUplink(int32_t left_total_ticks,
                           int32_t right_total_ticks,
                           float gyro_z_dps,
                           uint8_t status)
{
    uint8_t frame[FLYT_UPLINK_LEN];

    FlytProtocol_PackUplink(left_total_ticks,
                            right_total_ticks,
                            gyro_z_dps,
                            status,
                            frame);

    FlytSerial_SendBytes(frame, FLYT_UPLINK_LEN);
}

uint8_t FlytSerial_GetLatestCmd(FlytCmd_t *out_cmd)
{
    uint8_t has_cmd = 0;

    if (out_cmd == 0)
    {
        return 0;
    }

    __disable_irq();

    if (g_cmd_new_flag)
    {
        out_cmd->linear_mps = g_latest_cmd.linear_mps;
        out_cmd->angular_radps = g_latest_cmd.angular_radps;
        out_cmd->valid = g_latest_cmd.valid;
        g_cmd_new_flag = 0;
        has_cmd = 1;
    }

    __enable_irq();

    return has_cmd;
}

void USART1_IRQHandler(void)
{
    static uint8_t rx_buf[FLYT_DOWNLINK_LEN];
    static uint8_t rx_state = 0;
    static uint8_t rx_idx = 0;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART1);

        switch (rx_state)
        {
            case 0:
                if (data == FLYT_DOWN_STX1)
                {
                    rx_buf[0] = data;
                    rx_idx = 1;
                    rx_state = 1;
                }
                break;

            case 1:
                if (data == FLYT_DOWN_STX2)
                {
                    rx_buf[1] = data;
                    rx_idx = 2;
                    rx_state = 2;
                }
                else if (data == FLYT_DOWN_STX1)
                {
                    rx_buf[0] = data;
                    rx_idx = 1;
                    rx_state = 1;
                }
                else
                {
                    rx_state = 0;
                    rx_idx = 0;
                }
                break;

            case 2:
                rx_buf[rx_idx++] = data;

                if (rx_idx >= FLYT_DOWNLINK_LEN)
                {
                    FlytCmd_t cmd;
                    rx_state = 0;
                    rx_idx = 0;

                    if (FlytProtocol_ParseDownlink(rx_buf, &cmd))
                    {
                        g_latest_cmd.linear_mps = cmd.linear_mps;
                        g_latest_cmd.angular_radps = cmd.angular_radps;
                        g_latest_cmd.valid = 1;
                        g_cmd_new_flag = 1;
                    }
                }
                break;

            default:
                rx_state = 0;
                rx_idx = 0;
                break;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
