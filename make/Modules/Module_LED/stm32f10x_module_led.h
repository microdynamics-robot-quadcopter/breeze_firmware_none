/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_system_led.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Declare the led function
Others:      none
Function List:
             1. void LED_Init(void);
History:
<author>    <date>         <desc>
maksyuki    2016.12.20     Modify the module
myyerrol    2017.04.11     Format the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_LED_H__
#define __STM32F10X_MODULE_LED_H__

#include "stm32f10x.h"

#define LED_A_ON     GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LED_A_OFF    GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define LED_B_ON     GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LED_B_OFF    GPIO_ResetBits(GPIOA, GPIO_Pin_8)

#define LED_C_ON     GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define LED_C_OFF    GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define LED_D_ON     GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LED_D_OFF    GPIO_ResetBits(GPIOB, GPIO_Pin_3)

#define LED_A_TOGGLE GPIO_WriteBit(GPIOA, GPIO_Pin_11, \
                                  !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_11))
#define LED_B_TOGGLE GPIO_WriteBit(GPIOA, GPIO_Pin_8,  \
                                  !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8))
#define LED_C_TOGGLE GPIO_WriteBit(GPIOB, GPIO_Pin_1,  \
                                  !GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1))
#define LED_D_TOGGLE GPIO_WriteBit(GPIOB, GPIO_Pin_3,  \
                                  !GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3))

extern void LED_Init(void);

#endif
