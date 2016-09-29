/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_driver_tim.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.17
Description: declare the timer set operation
Others:      none
Function List:
             1. extern void TIM4_Init(u16 arr, u16 psc);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_TIM_H__
#define __STM32F10X_DRIVER_TIM_H__

#include "stm32f10x.h"

/*控制算法主程序定时器中断变量*/
extern volatile uint8_t loop20HZCnt;
extern volatile uint8_t loop50HZCnt;
extern volatile uint8_t loop100HZCnt;

extern void TIM4_Init(u16 arr, u16 psc); /*TIM4初始化*/

#endif
