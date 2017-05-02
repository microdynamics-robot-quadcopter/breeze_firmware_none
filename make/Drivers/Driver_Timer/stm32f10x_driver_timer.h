/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_timer.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.17
Description: Declare the timer set operation
Others:      none
Function List:
             1. void Timer_InitTIM3(u16 arr, u16 psc)
             2. void Timer_InitTIM4(u16 arr, u16 psc);
History:
<author>    <date>        <desc>
maksyuki    2016.12.03    Modify the module
myyerrol    2017.04.24    Format the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_TIM_H__
#define __STM32F10X_DRIVER_TIM_H__

#include <stdbool.h>
#include "stm32f10x.h"

// The interrupt variables of timer for main control.
extern vu16 timer_loop_cnt_10hz;
extern vu16 timer_loop_cnt_20hz;
extern vu16 timer_loop_cnt_50hz;
extern vu16 timer_loop_cnt_100hz;

// The interrupt flags of timer for main control.
extern bool timer_loop_flag_10hz;
extern bool timer_loop_flag_20hz;
extern bool timer_loop_flag_50hz;
extern bool timer_loop_flag_100hz;

extern void Timer_InitTIM3(u16 arr, u16 psc);
extern void Timer_InitTIM4(u16 arr, u16 psc);

#endif
