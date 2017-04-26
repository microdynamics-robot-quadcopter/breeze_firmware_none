/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_it.h
Author:      myyerrol
Version:     none
Create date: 2017.04.24
Description: Declare the Interrupt function
Others:      none
Function List:
History:
<author>    <date>        <desc>
myyerrol    2017.04.24    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_IT_H__
#define __STM32F10X_IT_H__

#include "stm32f10x.h"

// Current uptime for 1KHz systick timer.
extern vu32 it_systick_uptime;

extern void HardFault_Handler(void);
extern void SysTick_Handler(void);
extern void TIM3_IRQHandler(void);
extern void TIM4_IRQHandler(void);
extern void USART1_IRQHandler(void);

#endif
