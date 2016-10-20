/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_driver_tim.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.17
Description: implement the timer set operation
Others:      none
Function List:
             1. extern void TIM4_Init(u16 arr, u16 psc);
History:     none
*******************************************************************************/

#include "stm32f10x_driver_tim.h"
#include "stm32f10x_system_led.h"

volatile uint16_t loop10HZCnt  = 0;
volatile uint16_t loop20HZCnt  = 0;
volatile uint16_t loop50HZCnt  = 0;
volatile uint16_t loop100HZCnt = 0;

uint8_t loop10HZFlag;
uint8_t loop20HZFlag;
uint8_t loop50HZFlag;
uint8_t loop100HZFlag;

void TIM4_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_DeInit(TIM4);  /*复位定时器4*/

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

/*每1ms申请一次中断*/
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

        if (++loop100HZCnt * 100 >= 1000)
        {
            loop100HZCnt  = 0;
            loop100HZFlag = 1;
        }
        if (++loop50HZCnt * 50 >= 1000)
        {
            loop50HZCnt  = 0;
            loop50HZFlag = 1;
        }
        if (++loop10HZCnt * 10 >= 1000)
        {
            loop10HZCnt  = 0;
            loop10HZFlag = 1;
        }
    }
}
