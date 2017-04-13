/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_delay.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.02
Description: Implement the time delay function
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

#include "stm32f10x_driver_delay.h"

// Cycles per millisecond.
static vu32 tick_us = 0;
// Current uptime for 1KHz systick timer.
extern vu32 systick_uptime;

void Delay_Init(void)
{
    RCC_ClocksTypeDef RCC_ClocksStructure;
    RCC_GetClocksFreq(&RCC_ClocksStructure);
    tick_us = RCC_ClocksStructure.SYSCLK_Frequency / 1000000;
}

void Delay_TimeUs(u32 n_us)
{
    u32 t = Delay_GetRuntimeUs();
    while(Delay_GetRuntimeUs() - t < n_us);
}

void Delay_TimeMs(u16 n_ms)
{
    u32 t = Delay_GetRuntimeUs();
    while(Delay_GetRuntimeUs() - t < n_ms * 1000);
}

u32 Delay_GetRuntimeUs(void)
{
    register u32 milliseconds;
    register u32 cycle_count;

    do
    {
        milliseconds = systick_uptime;
        cycle_count  = SysTick->VAL;
    }
    while (milliseconds != systick_uptime);

    return (milliseconds * 1000) + (tick_us * 1000 - cycle_count) / tick_us;
}

u32 Delay_GetRuntimeMs(void)
{
    return systick_uptime;
}
