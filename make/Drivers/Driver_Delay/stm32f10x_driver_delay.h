/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_delay.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.02
Description: Declare the time delay function
Others:      none
Function List:
             1. void Delay_Init(void);
             2. void Delay_TimeMs(u16 n_ms);
             3. void Delay_TimeUs(u32 n_us);
             4. u32  Delay_GetRuntimeMs(void);
             5. u32  Delay_GetRuntimeUs(void);
History:
<author>    <date>        <desc>
maksyuki    2016.12.15    Modify the module
myyerrol    2017.04.13    Format the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_DELAY_H__
#define __STM32F10X_DRIVER_DELAY_H__

#include "stm32f10x.h"

extern void Delay_Init(void);
extern void Delay_TimeMs(u16 n_ms);
extern void Delay_TimeUs(u32 n_us);
extern u32  Delay_GetRuntimeMs(void);
extern u32  Delay_GetRuntimeUs(void);

#endif
