/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_pwm.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.8.4
Description: implement the PWM operation function
Others:      none
Function List:
             1. void PWM_Init(void);
             2. void PWM_MotorFlash(int16_t MOTO1_PWM, int16_t MOTO2_PWM,
                                    int16_t MOTO3_PWM, int16_t MOTO4_PWM);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.04  modify the module
*******************************************************************************/

#include "stm32f10x_driver_pwm.h"

void PWM_Init(void)
{
    uint16_t                 PrescalerValue = 0;    /* Control the frequency rate of PWM */
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Set GPIO function */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_DeInit(TIM2);

    /* Configure timer */
    PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
    TIM_TimeBaseStructure.TIM_Period        = 999;                 /* The upper limit of counter */
    TIM_TimeBaseStructure.TIM_Prescaler     = PrescalerValue;      /* The timer fractional frequency of PWM */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Configure the model of the TIM2 is PWM */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_Cmd(TIM2, ENABLE);
}

void PWM_MotorFlash(int16_t MOTO1_PWM, int16_t MOTO2_PWM, int16_t MOTO3_PWM, int16_t MOTO4_PWM)
{
    if (MOTO1_PWM < 0) MOTO1_PWM = 0;
    if (MOTO2_PWM < 0) MOTO2_PWM = 0;
    if (MOTO3_PWM < 0) MOTO3_PWM = 0;
    if (MOTO4_PWM < 0) MOTO4_PWM = 0;
    if (MOTO1_PWM > PWM_MAXVALUE) MOTO1_PWM = PWM_MAXVALUE;
    if (MOTO2_PWM > PWM_MAXVALUE) MOTO2_PWM = PWM_MAXVALUE;
    if (MOTO3_PWM > PWM_MAXVALUE) MOTO3_PWM = PWM_MAXVALUE;
    if (MOTO4_PWM > PWM_MAXVALUE) MOTO4_PWM = PWM_MAXVALUE;

    TIM_SetCompare1(TIM2, MOTO1_PWM);
    TIM_SetCompare2(TIM2, MOTO2_PWM);
    TIM_SetCompare3(TIM2, MOTO3_PWM);
    TIM_SetCompare4(TIM2, MOTO4_PWM);
}
