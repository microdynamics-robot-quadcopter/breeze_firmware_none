/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_system_led.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: declare the led function
Others:      none
Function List:
             1. void LED_Init(void);
             2. void LED_test(int flag);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.20  modify the module
*******************************************************************************/

#ifndef __STM32F10X_SYSTEM_LED_H__
#define __STM32F10X_SYSTEM_LED_H__

#include "stm32f10x.h"

#define LedA_On    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LedA_Off   GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define LedB_On    GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LedB_Off   GPIO_ResetBits(GPIOA, GPIO_Pin_8)

#define LedC_On    GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define LedC_Off   GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define LedD_On    GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LedD_Off   GPIO_ResetBits(GPIOB, GPIO_Pin_3)

#define LEDC_troggle GPIO_WriteBit(GPIOB, GPIO_Pin_1, !GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1))
#define LEDB_troggle GPIO_WriteBit(GPIOA, GPIO_Pin_8, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8))
#define LEDA_troggle GPIO_WriteBit(GPIOA, GPIO_Pin_11, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_11))

extern void LED_Init(void);     /* Initialize led */
extern void LED_test(int flag);

#endif
