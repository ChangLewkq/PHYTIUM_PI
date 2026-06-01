#include "stm32f10x.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "Delay.h"

#define MPU6050_ADDRESS             0xD0
#define MPU6050_INIT_RETRY          5

/*
 * WHO_AM_I 常见返回值：
 * - 0x68: MPU6050
 * - 0x69: MPU6050 某些 AD0/兼容情况
 * - 0x70: MPU6500 / 部分 MPU6050 兼容模块
 * - 0x71: MPU9250 / ICM 兼容模块常见值
 *
 * 这些芯片的 gyro_z/accel 基础寄存器和量程设置基本兼容当前用法。
 */
static uint8_t MPU6050_IsValidIDValue(uint8_t id)
{
    if (id == 0x68 || id == 0x69 || id == 0x70 || id == 0x71)
    {
        return 1;
    }

    return 0;
}

static uint8_t MPU6050_WriteRegChecked(uint8_t RegAddress, uint8_t Data)
{
    uint8_t ack;

    MyI2C_Start();

    MyI2C_SendByte(MPU6050_ADDRESS);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        return 0;
    }

    MyI2C_SendByte(RegAddress);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        return 0;
    }

    MyI2C_SendByte(Data);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        return 0;
    }

    MyI2C_Stop();
    return 1;
}

static uint8_t MPU6050_ReadRegChecked(uint8_t RegAddress, uint8_t *Data)
{
    uint8_t ack;

    if (Data == 0)
    {
        return 0;
    }

    *Data = 0xFF;

    MyI2C_Start();

    MyI2C_SendByte(MPU6050_ADDRESS);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        return 0;
    }

    MyI2C_SendByte(RegAddress);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        return 0;
    }

    MyI2C_Start();

    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        return 0;
    }

    *Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return 1;
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    (void)MPU6050_WriteRegChecked(RegAddress, Data);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data = 0xFF;

    if (!MPU6050_ReadRegChecked(RegAddress, &Data))
    {
        MyI2C_BusRecover();
        Delay_ms(2);
        (void)MPU6050_ReadRegChecked(RegAddress, &Data);
    }

    return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    uint8_t i;
    uint8_t ack;

    if (DataArray == 0 || Count == 0)
    {
        return;
    }

    for (i = 0; i < Count; i++)
    {
        DataArray[i] = 0xFF;
    }

    MyI2C_Start();

    MyI2C_SendByte(MPU6050_ADDRESS);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        MyI2C_BusRecover();
        return;
    }

    MyI2C_SendByte(RegAddress);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        MyI2C_BusRecover();
        return;
    }

    MyI2C_Start();

    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    ack = MyI2C_ReceiveAck();
    if (ack)
    {
        MyI2C_Stop();
        MyI2C_BusRecover();
        return;
    }

    for (i = 0; i < Count; i++)
    {
        DataArray[i] = MyI2C_ReceiveByte();

        if (i < Count - 1)
        {
            MyI2C_SendAck(0);
        }
        else
        {
            MyI2C_SendAck(1);
        }
    }

    MyI2C_Stop();
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

uint8_t MPU6050_IsValidID(uint8_t id)
{
    return MPU6050_IsValidIDValue(id);
}

uint8_t MPU6050_IsOnline(void)
{
    return MPU6050_IsValidIDValue(MPU6050_GetID()) ? 1 : 0;
}

void MPU6050_Init(void)
{
    uint8_t i;
    uint8_t id;

    MyI2C_Init();

    Delay_ms(100);

    for (i = 0; i < MPU6050_INIT_RETRY; i++)
    {
        MyI2C_BusRecover();
        Delay_ms(10);

        MPU6050_WriteRegChecked(MPU6050_PWR_MGMT_1, 0x80);
        Delay_ms(100);

        MPU6050_WriteRegChecked(MPU6050_PWR_MGMT_1, 0x01);
        Delay_ms(10);

        MPU6050_WriteRegChecked(MPU6050_PWR_MGMT_2, 0x00);
        Delay_ms(5);

        MPU6050_WriteRegChecked(MPU6050_SMPLRT_DIV, 0x07);
        MPU6050_WriteRegChecked(MPU6050_CONFIG, 0x00);
        MPU6050_WriteRegChecked(MPU6050_GYRO_CONFIG, 0x18);
        MPU6050_WriteRegChecked(MPU6050_ACCEL_CONFIG, 0x18);

        Delay_ms(20);

        id = MPU6050_GetID();

        if (MPU6050_IsValidIDValue(id))
        {
            return;
        }
    }
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];

    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);

    *AccX = (int16_t)((Data[0] << 8) | Data[1]);
    *AccY = (int16_t)((Data[2] << 8) | Data[3]);
    *AccZ = (int16_t)((Data[4] << 8) | Data[5]);

    *GyroX = (int16_t)((Data[8] << 8) | Data[9]);
    *GyroY = (int16_t)((Data[10] << 8) | Data[11]);
    *GyroZ = (int16_t)((Data[12] << 8) | Data[13]);
}

float MPU6050_GetGyroZ(void)
{
    uint8_t DataH;
    uint8_t DataL;
    int16_t GyroZ;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);

    if (DataH == 0xFF && DataL == 0xFF)
    {
        return 0.0f;
    }

    GyroZ = (int16_t)((DataH << 8) | DataL);

    return GyroZ / 16.4f;
}
