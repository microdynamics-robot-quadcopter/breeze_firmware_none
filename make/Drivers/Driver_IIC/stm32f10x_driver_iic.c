/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_iic.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.03
Description: Implement the IIC operation function
Others:      none
Function List:
             1. void IIC_Init(void);
             2. void IIC_Start(void);
             3. void IIC_Stop(void);
             4. void IIC_SendByte(u8 byte);
             5. u8 IIC_ReadByte(u8 ack);
             6. u8 IIC_WaitAck(void);
             7. void IIC_Ack(void);
             8. void IIC_NAck(void);
             9. u8 IIC_ReadOneByte(u8 IIC_Addr, u8 addr);
             10.u8 IICReadBytes(u8 dev, u8 reg, u8 length, u8 *data);
             11.u8 IICWriteBytes(u8 dev, u8 reg, u8 length, u8 *data);
             12.u8 IICReadByte(u8 dev, u8 reg, u8 *data);
             13.u8 IICWriteByte(u8 dev, u8 reg, u8 data);
             14.u8 IICWriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data);
             15.u8 IICWriteBit(u8 dev, u8 reg, u8 bitNum, u8 data);
History:
<author>    <date>        <desc>
maksyuki    2016.11.30    Modify the module
myyerrol    2017.04.23    Format the module
*******************************************************************************/

#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x.h"

void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void IIC_SendAckSignal(void)
{
    IIC_SCL = 0;
    IIC_SDA_OUT();
    IIC_SDA = 0;
    Delay_TimeUs(1);
    IIC_SCL = 1;
    Delay_TimeUs(1);
    IIC_SCL = 0;
}

void IIC_SendStartSignal(void)
{
    IIC_SDA_OUT();
    IIC_SDA = 1;
    IIC_SCL = 1;
    Delay_TimeUs(4);
    IIC_SDA = 0;
    Delay_TimeUs(4);
    IIC_SCL = 0;
}

void IIC_SendStopSignal(void)
{
    IIC_SDA_OUT();
    IIC_SCL = 0;
    IIC_SDA = 0;
    Delay_TimeUs(4);
    IIC_SCL = 1;
    IIC_SDA = 1;
    Delay_TimeUs(4);
}

void IIC_SendNAckSignal(void)
{
    IIC_SCL = 0;
    IIC_SDA_OUT();
    IIC_SDA = 1;
    Delay_TimeUs(1);
    IIC_SCL = 1;
    Delay_TimeUs(1);
    IIC_SCL = 0;
}

u8 IIC_ReadByte(u8 iic_addr, u8 reg_addr)
{
    u8 byte = 0;

    IIC_SendStartSignal();
    IIC_WriteOneByte(iic_addr);
    byte++;
    IIC_WaitAckSignal();
    IIC_WriteOneByte(reg_addr);
    byte++;
    IIC_WaitAckSignal();
    IIC_SendStartSignal();
    IIC_WriteOneByte(iic_addr + 1);
    byte++;
    IIC_WaitAckSignal();
    byte = IIC_ReadOneByte(0);
    IIC_SendStopSignal();

    return byte;
}

u8 IIC_ReadBytes(u8 dev_addr, u8 reg_addr, u8 byte_nums, u8 *data)
{
    u8 i = 0;
    u8 temp;

    IIC_SendStartSignal();
    IIC_WriteOneByte(dev_addr);
    IIC_WaitAckSignal();
    IIC_WriteOneByte(reg_addr);
    IIC_WaitAckSignal();
    IIC_SendStartSignal();
    IIC_WriteOneByte(dev_addr + 1);
    IIC_WaitAckSignal();

    for (i = 0; i < byte_nums; i++)
    {
        if (i != (byte_nums - 1))
        {
            temp = IIC_ReadOneByte(1);
        }
        else
        {
            temp = IIC_ReadOneByte(0);
        }
        data[i] = temp;
    }

    IIC_SendStopSignal();

    return i;
}

u8 IIC_ReadOneByte(u8 ack)
{
    u8 i;
    u8 byte = 0;

    IIC_SDA_IN();

    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        Delay_TimeUs(1);
        IIC_SCL = 1;
        byte  <<= 1;
        if (IIC_SDA_READ)
        {
            byte++;
        }
        Delay_TimeUs(1);
    }

    if (ack)
    {
        IIC_SendAckSignal();
    }
    else
    {
        IIC_SendNAckSignal();
    }

    return byte;
}

u8 IIC_WaitAckSignal(void)
{
    u8 error_time = 0;

    IIC_SDA_IN();
    IIC_SDA = 1;
    Delay_TimeUs(1);
    IIC_SCL = 1;
    Delay_TimeUs(1);

    while (IIC_SDA_READ)
    {
        error_time++;
        if (error_time > 50)
        {
            IIC_SendStopSignal();
            return 1;
        }
        Delay_TimeUs(1);
    }

    IIC_SCL = 0;

    return 0;
}

u8 IIC_WriteBit(u8 dev_addr, u8 reg_addr, u8 bit_index, u8 data)
{
    u8 byte;

    byte = IIC_ReadByte(dev_addr, reg_addr);
    byte = (data != 0) ? (byte | (1 << bit_index)) : (byte & ~(1 << bit_index));

    return IIC_WriteByte(dev_addr, reg_addr, byte);
}

u8 IIC_WriteBits(u8 dev_addr, u8 reg_addr, u8 bit_start, u8 bit_len, u8 data)
{
    u8 byte;
    u8 mask;

    byte = IIC_ReadByte(dev_addr, reg_addr);
    mask = (0xFF << (bit_start + 1)) | 0xFF >> ((8 - bit_start) + bit_len - 1);
    data <<= (8 - bit_len);
    data >>= (7 - bit_start);
    byte &= mask;
    byte |= data;

    return IIC_WriteByte(dev_addr, reg_addr, byte);
}

u8 IIC_WriteByte(u8 dev_addr, u8 reg_addr, u8 data)
{
    return IIC_WriteBytes(dev_addr, reg_addr, 1, &data);
}


u8 IIC_WriteBytes(u8 dev_addr, u8 reg_addr, u8 byte_nums, u8 *data)
{
    u8 i = 0;

    IIC_SendStartSignal();
    IIC_WriteOneByte(dev_addr);
    IIC_WaitAckSignal();
    IIC_WriteOneByte(reg_addr);
    IIC_WaitAckSignal();

    for (i = 0; i < byte_nums; i++)
    {
        IIC_WriteOneByte(data[i]);
        IIC_WaitAckSignal();
    }

    IIC_SendStopSignal();

    return 1;
}

u8 IIC_WriteOneByte(u8 byte)
{
    u8 i;

    IIC_SDA_OUT();
    IIC_SCL = 0;

    for (i = 0; i < 8; i++)
    {
        IIC_SDA = (byte & 0x80) >> 7;
        byte <<= 1;
        Delay_TimeUs(1);
        IIC_SCL = 1;
        Delay_TimeUs(1);
        IIC_SCL = 0;
        Delay_TimeUs(1);
    }
}
