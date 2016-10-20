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
extern volatile uint16_t loop10HzCnt;
extern volatile uint16_t loop20HzCnt;
extern volatile uint16_t loop50HzCnt;
extern volatile uint16_t loop100HzCnt;

/*控制算法主程序定时器中断标志*/
extern uint8_t loop10HzFlag;
extern uint8_t loop20HzFlag;
extern uint8_t loop50HzFlag;
extern uint8_t loop100HzFlag;

extern void TIM4_Init(u16 arr, u16 psc); /*TIM4初始化*/

#endif
