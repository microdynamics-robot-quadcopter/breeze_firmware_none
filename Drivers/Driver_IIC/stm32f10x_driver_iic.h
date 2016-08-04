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
			 4. void IIC_Send_Byte(u8 txd);
			 5. u8 IIC_Read_Byte(u8 ack);
			 6. u8 IIC_Wait_Ack(void); 
			 7. void IIC_Ack(void); 
			 8. void IIC_NAck(void);  
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_IIC_H__
#define __STM32F10X_DRIVER_IIC_H__

#include "stm32f10x.h"

/*the setting of IO interfaces direction*/
#define SDA_IN()  {GPIOA->CRH &= 0XFFF0FFFF; GPIOA->CRH |= 8 << 16;}
#define SDA_OUT() {GPIOA->CRH &= 0XFFF0FFFF; GPIOA->CRH |= 3 << 16;}

/*IO interfaces operation*/ 
#define IIC_SCL_H    GPIO_SetBits(GPIOA, GPIO_Pin_11)           /*SCL_H*/
#define IIC_SCL_L    GPIO_ResetBits(GPIOA, GPIO_Pin_11)         /*SCL_L*/
#define IIC_SDA_H    GPIO_SetBits(GPIOA, GPIO_Pin_12)           /*SDA_H*/
#define IIC_SDA_L    GPIO_ResetBits(GPIOA, GPIO_Pin_12)         /*SDA_L*/
#define READ_SDA  	 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)  /*Input SDA*/

/*IIC operation*/
void IIC_Init(void);            /*Initialize IIC*/
void IIC_Start(void);           /*Send start signal*/ 
void IIC_Stop(void);            /*Send end signal*/
void IIC_Send_Byte(u8 txd);     /*IIC sends one byte*/
u8 IIC_Read_Byte(u8 ack);       /*IIC reads one byte*/
u8 IIC_Wait_Ack(void);          /*IIC waits ACK signal*/
void IIC_Ack(void);             /*IIC sends ACK signal*/
void IIC_NAck(void);            /*IIC doesn't send ACK signal*/

#endif
