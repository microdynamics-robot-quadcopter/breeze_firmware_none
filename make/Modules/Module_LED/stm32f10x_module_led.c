/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_system_led.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Implement the led function
Others:      none
Function List:
             1. void LED_Init(void);
             2. void LED_SetLight(LED_State LED_A, LED_State LED_B,
                                  LED_State LED_C, LED_State LED_D)
History:
<author>    <date>        <desc>
maksyuki    2016.12.20    Modify the
myyerrol    2017.04.11    Format the module
*******************************************************************************/

#include "stm32f10x_module_led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // When hardware disables the JATG, don't disable the SWD meanwhile.
    // Otherwise the chip will be damaged!!!
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    LED_A_OFF;
    LED_B_OFF;
    LED_C_OFF;
    LED_D_OFF;
}

void LED_SetLight(LED_State LED_A, LED_State LED_B, LED_State LED_C,
                  LED_State LED_D)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, LED_A);
    GPIO_WriteBit(GPIOA, GPIO_Pin_8,  LED_B);
    GPIO_WriteBit(GPIOB, GPIO_Pin_1,  LED_C);
    GPIO_WriteBit(GPIOB, GPIO_Pin_3,  LED_D);
}
