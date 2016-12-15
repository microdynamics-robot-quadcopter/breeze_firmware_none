/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_delay.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.02
Description: declare the time delay function
Others:      none
Function List:
             1. void delay_init(void);
             2. void delay_us(u32 nus);
             3. void delay_ms(u16 nms);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.15  modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_DELAY_H__
#define __STM32F10X_DRIVER_DELAY_H__

#include "stm32f10x.h"

extern char SysClock;          /* unit: MHz */

extern char SystemClock_HSE(u8 PLL);
extern void delay_init(void);  /* Initializes the delay function */
extern void delay_us(u32 nus); /* Set time with microsecond precision */
extern void delay_ms(u16 nms); /* Set time with millisecond precision */

#endif
