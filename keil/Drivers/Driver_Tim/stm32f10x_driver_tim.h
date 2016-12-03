/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_tim.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.8.17
Description: declare the timer set operation
Others:      none
Function List:
             1. void TIM4_Init(u16 arr, u16 psc);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.03  modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_TIM_H__
#define __STM32F10X_DRIVER_TIM_H__

#include "stm32f10x.h"

/* The interrupt variables of timer for main control */
extern volatile uint16_t loop10HzCnt;
extern volatile uint16_t loop20HzCnt;
extern volatile uint16_t loop50HzCnt;
extern volatile uint16_t loop100HzCnt;

/* The interrupt flags of timer for main control */
extern uint8_t loop10HzFlag;
extern uint8_t loop20HzFlag;
extern uint8_t loop50HzFlag;
extern uint8_t loop100HzFlag;

extern void TIM4_Init(u16 arr, u16 psc); /* TIM4 initialization */

#endif
