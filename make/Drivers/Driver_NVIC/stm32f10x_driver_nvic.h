/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_nvic.h
Author:      myyerrol
Version:     none
Create date: 2017.04.14
Description: Declare the NVIC function
Others:      none
Function List:
             1. void NVIC_InitUSART(void);
             2. void NVIC_InitUSART1(void);
History:
<author>    <date>        <desc>
myyerrol    2017.04.14    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_NVIC_H__
#define __STM32F10X_DRIVER_NVIC_H__

#include "stm32f10x.h"

extern void NVIC_InitUSART(void);
extern void NVIC_InitUSART1(void);

#endif
