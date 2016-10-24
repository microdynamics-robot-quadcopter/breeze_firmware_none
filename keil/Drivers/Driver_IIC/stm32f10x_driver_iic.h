/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_driver_iic.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.3
Description: define the IIC operation and declare the IIC operation function
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
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_IIC_H__
#define __STM32F10X_DRIVER_IIC_H__

#include "stm32f10x_driver_sys.h"
#include "stm32f10x.h"

/*the setting of IO interfaces direction*/
/*SCL-->PB6*/
/*SDA-->PB7*/
#define SDA_IN()  {GPIOB->CRL &= 0X0FFFFFFF; GPIOB->CRL |= 0x80000000;}
#define SDA_OUT() {GPIOB->CRL &= 0X0FFFFFFF; GPIOB->CRL |= 0x30000000;}

/*IO interfaces operation*/
#define IIC_SCL  PBout(6)
#define IIC_SDA  PBout(7)
#define READ_SDA PBin(7)

/*IIC operation*/
extern void IIC_Init(void);            /*Initialize IIC*/
extern void IIC_Start(void);           /*Send start signal*/
extern void IIC_Stop(void);            /*Send end signal*/
extern void IIC_SendByte(u8 txd);      /*IIC sends one byte*/
extern u8 IIC_ReadByte(u8 ack);        /*IIC reads one byte*/
extern u8 IIC_WaitAck(void);           /*IIC waits ACK signal*/
extern void IIC_Ack(void);             /*IIC sends ACK signal*/
extern void IIC_NAck(void);            /*IIC doesn't send ACK signal*/

extern u8 IIC_ReadOneByte(u8 IIC_Addr, u8 addr);
extern u8 IICReadBytes(u8 dev, u8 reg, u8 length, u8 *data);
extern u8 IICWriteBytes(u8 dev, u8 reg, u8 length, u8 *data);
extern u8 IICReadByte(u8 dev, u8 reg, u8 *data);
extern u8 IICWriteByte(u8 dev, u8 reg, u8 data);
extern u8 IICWriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data);
extern u8 IICWriteBit(u8 dev, u8 reg, u8 bitNum, u8 data);

#endif
