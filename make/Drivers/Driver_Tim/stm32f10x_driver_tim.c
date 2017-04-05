/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_tim.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.17
Description: implement the timer set operation
Others:      none
Function List:
             1. void TIM4_Init(u16 arr, u16 psc);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.03  modify the module
*******************************************************************************/

#include "stm32f10x_driver_tim.h"
#include "stm32f10x_module_led.h"

volatile uint16_t loop10HzCnt  = 0;
volatile uint16_t loop20HzCnt  = 0;
volatile uint16_t loop50HzCnt  = 0;
volatile uint16_t loop100HzCnt = 0;

uint8_t loop10HzFlag  = 0;
uint8_t loop20HzFlag  = 0;
uint8_t loop50HzFlag  = 0;
uint8_t loop100HzFlag = 0;

void TIM4_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period        = arr;
    TIM_TimeBaseStructure.TIM_Prescaler     = psc - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM4, ENABLE);
}

/* Request interrupts per millisecond */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

        if (++loop100HzCnt * 100 >= 1000)  /* Overflow time is 10ms */
        {
            loop100HzCnt  = 0;
            loop100HzFlag = 1;
        }
        if (++loop50HzCnt * 50 >= 1000)    /* Overflow time is 20ms */
        {
            loop50HzCnt  = 0;
            loop50HzFlag = 1;
        }
        if (++loop10HzCnt * 10 >= 1000)    /* Overflow time is 100ms */
        {
            loop10HzCnt  = 0;
            loop10HzFlag = 1;
        }
    }
}
