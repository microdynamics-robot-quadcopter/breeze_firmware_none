/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_driver_usart.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.15
Description: define the serial port operation
Others:      none
Function List:
             1. extern void usart_init(u32 bound);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_USART_H__
#define __STM32F10X_DRIVER_USART_H__

#include "stm32f10x.h"

#define USART_REC_LEN 200               /*定义最大接收字节数*/
#define EN_USART_RX   1                 /*使能串口接收*/

extern u8 USART_RX_BUF[USART_REC_LEN];  /*接收缓冲，最大USART_REC_LEN个字节，最后一个字节为换行符*/
extern u16 USART_RX_STA;                /*接收状态标记*/

extern void usart_init(u32 bound);

#endif
