/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_iic.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.03
Description: implement the IIC operation function
Others:      none
Function List:
             1. void IIC_Init(void);
             2. void IIC_Start(void);
             3. void IIC_Stop(void);
             4. void IIC_SendByte(u8 txd);
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
1. <author>    <date>         <desc>
   maksyuki  2016.11.30  modify the module
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

void IIC_Start(void)
{
    SDA_OUT();
    IIC_SDA = 1;
    IIC_SCL = 1;
    Delay_TimeUs(4);
    IIC_SDA = 0;
    Delay_TimeUs(4);
    IIC_SCL = 0;
}

void IIC_Stop(void)
{
    SDA_OUT();
    IIC_SCL = 0;
    IIC_SDA = 0;
    Delay_TimeUs(4);
    IIC_SCL = 1;
    IIC_SDA = 1;
    Delay_TimeUs(4);
}

u8 IIC_WaitAck(void)
{
    u8 ucErrTime = 0;
    SDA_IN();
    IIC_SDA = 1;
    Delay_TimeUs(1);
    IIC_SCL = 1;
    Delay_TimeUs(1);

    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
            IIC_Stop();
            return 1;
        }
        Delay_TimeUs(1);
    }
    IIC_SCL = 0;
    return 0;
}

void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    Delay_TimeUs(1);
    IIC_SCL = 1;
    Delay_TimeUs(1);
    IIC_SCL = 0;
}

void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    Delay_TimeUs(1);
    IIC_SCL = 1;
    Delay_TimeUs(1);
    IIC_SCL = 0;
}

void IIC_SendByte(u8 txd)
{
    u8 i;
    SDA_OUT();
    IIC_SCL = 0;

    for (i = 0; i < 8; i++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        Delay_TimeUs(1);
        IIC_SCL = 1;
        Delay_TimeUs(1);
        IIC_SCL = 0;
        Delay_TimeUs(1);
    }
}

u8 IIC_ReadByte(u8 ack)
{
    u8 i;
    u8 receive = 0;
    SDA_IN();

    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        Delay_TimeUs(1);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
        {
            receive++;
        }
        Delay_TimeUs(1);
    }

    if (ack)
    {
        IIC_Ack();
    }
    else
    {
        IIC_NAck();
    }
    return receive;
}

u8 IIC_ReadOneByte(u8 IIC_Addr, u8 addr)
{
    u8 res = 0;
    IIC_Start();
    IIC_SendByte(IIC_Addr);
    res++;
    IIC_WaitAck();
    IIC_SendByte(addr);
    res++;
    IIC_WaitAck();
    IIC_Start();
    IIC_SendByte(IIC_Addr + 1);
    res++;
    IIC_WaitAck();
    res = IIC_ReadByte(0);
    IIC_Stop();
    return res;
}

u8 IICReadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
    u8 temp;
    IIC_Start();
    IIC_SendByte(dev);
    IIC_WaitAck();
    IIC_SendByte(reg);
    IIC_WaitAck();
    IIC_Start();
    IIC_SendByte(dev + 1);
    IIC_WaitAck();

    for (count = 0; count < length; count++)
    {
        if (count != (length - 1))
        {
            temp = IIC_ReadByte(1);
        }
        else
        {
            temp = IIC_ReadByte(0);
        }
        data[count] = temp;
    }
    IIC_Stop();
    return count;
}

u8 IICWriteBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
    IIC_Start();
    IIC_SendByte(dev);
    IIC_WaitAck();
    IIC_SendByte(reg);
    IIC_WaitAck();

    for (count = 0; count < length; count++)
    {
        IIC_SendByte(data[count]);
        IIC_WaitAck();
    }
    IIC_Stop();

    return 1;
}

u8 IICReadByte(u8 dev, u8 reg, u8 *data)
{
    *data = IIC_ReadOneByte(dev, reg);
    return 1;
}

u8 IICWriteByte(u8 dev, u8 reg, u8 data)
{
    return IICWriteBytes(dev, reg, 1, &data);
}

u8 IICWriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{
    u8 b;
    if (IICReadByte(dev, reg, &b) != 0)
    {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICWriteByte(dev, reg, b);
    }
    else
    {
        return 0;
    }
}

u8 IICWriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    IICReadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICWriteByte(dev, reg, b);
}
