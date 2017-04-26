/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_motor.c
Author:      myyerrol
Version:     none
Create date: 2017.04.23
Description: Implement the motor operation function
Others:      none
Function List:
             1. void Motor_Init(void);
             2. void Motor_SetPWM(s16 motor1_pwm, s16 motor2_pwm,
                                  s16 motor3_pwm, s16 motor4_pwm)
History:
<author>    <date>        <desc>
myyerrol    2017.04.23    Modify the module
*******************************************************************************/

#include "stm32f10x_module_motor.h"

void Motor_Init(void)
{
    u16                     prescaler = (u16)(SystemCoreClock / 24000000) - 1;
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |
                                    GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period        = 999;
    TIM_TimeBaseStructure.TIM_Prescaler     = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

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

void Motor_SetPWM(s16 motor1_pwm, s16 motor2_pwm, s16 motor3_pwm,
                  s16 motor4_pwm)
{
    if (motor1_pwm <= 0)
    {
        motor1_pwm = 0;
    }
    if (motor2_pwm <= 0)
    {
        motor2_pwm = 0;
    }
    if (motor3_pwm <= 0)
    {
        motor3_pwm = 0;
    }
    if (motor4_pwm <= 0)
    {
        motor4_pwm = 0;
    }
    if (motor1_pwm >= MOTOR_PWM_MAXVALUE)
    {
        motor1_pwm = MOTOR_PWM_MAXVALUE;
    }
    if (motor2_pwm >= MOTOR_PWM_MAXVALUE)
    {
        motor2_pwm = MOTOR_PWM_MAXVALUE;
    }
    if (motor3_pwm >= MOTOR_PWM_MAXVALUE)
    {
        motor3_pwm = MOTOR_PWM_MAXVALUE;
    }
    if (motor4_pwm >= MOTOR_PWM_MAXVALUE)
    {
        motor4_pwm = MOTOR_PWM_MAXVALUE;
    }

    TIM_SetCompare1(TIM2, motor1_pwm);
    TIM_SetCompare2(TIM2, motor2_pwm);
    TIM_SetCompare3(TIM2, motor3_pwm);
    TIM_SetCompare4(TIM2, motor4_pwm);
}
