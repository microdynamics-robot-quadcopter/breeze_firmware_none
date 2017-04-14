/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_nvic.c
Author:      myyerrol
Version:     none
Create date: 2017.04.14
Description: Implement the NVIC function
Others:      none
Function List:
             1. void NVIC_InitUSART(void);
             2. void NVIC_InitUSART1(void);
History:
<author>    <date>        <desc>
myyerrol    2017.04.14    Modify the module
*******************************************************************************/

#include "stm32f10x_driver_nvic.h"

void NVIC_InitUSART(void)
{
    NVIC_InitUSART1();
}

void NVIC_InitUSART1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
