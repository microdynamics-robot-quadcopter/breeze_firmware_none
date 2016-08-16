/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_system_delay.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.2
Description: declare the time delay function
Others:      none
Function List:
             1. extern void delay_init(void);
             2. extern void delay_us(u32 nus);
             3. extern void delay_ms(u16 nms);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_SYSTEM_DELAY_H__
#define __STM32F10X_SYSTEM_DELAY_H__

#include "stm32f10x.h"

extern void delay_init(void);  /*Initializes the delay function*/
extern void delay_us(u32 nus); /*Set time with microsecond precision*/
extern void delay_ms(u16 nms); /*Set time with millisecond precision*/

#endif
