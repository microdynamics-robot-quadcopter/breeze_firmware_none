/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_iic.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.03
Description: Define the IIC operation and declare the IIC operation function
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
<author>    <date>        <desc>
maksyuki    2016.11.30    Modify the module
myyerrol    2017.04.23    Format the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_IIC_H__
#define __STM32F10X_DRIVER_IIC_H__

#include "stm32f10x.h"
#include "stm32f10x_driver_sys.h"

#define IIC_SDA_IN()          \
{                             \
    GPIOB->CRL &= 0X0FFFFFFF; \
    GPIOB->CRL |= 0x80000000; \
}

#define IIC_SDA_OUT()         \
{                             \
    GPIOB->CRL &= 0X0FFFFFFF; \
    GPIOB->CRL |= 0x30000000; \
}

#define IIC_SCL      PBout(6)
#define IIC_SDA      PBout(7)
#define IIC_SDA_READ PBin(7)

extern void IIC_Init(void);
extern void IIC_SendAckSignal(void);
extern void IIC_SendStartSignal(void);
extern void IIC_SendStopSignal(void);
extern void IIC_SendNAckSignal(void);
extern u8   IIC_ReadByte(u8 iic_addr, u8 reg_addr);
extern u8   IIC_ReadBytes(u8 dev_addr, u8 reg_addr, u8 byte_nums, u8 *data);
extern u8   IIC_ReadOneByte(u8 ack);
extern u8   IIC_WaitAckSignal(void);

extern u8   IIC_WriteBit(u8 dev_addr, u8 reg_addr, u8 bit_index, u8 data);
extern u8   IIC_WriteBits(u8 dev_addr, u8 reg_addr, u8 bit_start, u8 bit_len,
                          u8 data);
extern u8   IIC_WriteByte(u8 dev_addr, u8 reg_addr, u8 data);
extern u8   IIC_WriteBytes(u8 dev_addr, u8 reg_addr, u8 byte_nums, u8 *data);
extern u8   IIC_WriteOneByte(u8 byte);




#endif
