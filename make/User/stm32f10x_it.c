/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_it.c
Author:      myyerrol
Version:     none
Create date: 2017.04.24
Description: Implement the Interrupt function
Others:      none
Function List:
History:
<author>    <date>        <desc>
myyerrol    2017.04.24    Modify the module
*******************************************************************************/

#include "stm32f10x_it.h"
#include "stm32f10x_driver_timer.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_led.h"

vu32 it_systick_uptime = 0;

void HardFault_Handler(void)
{
}

void SysTick_Handler(void)
{
    it_systick_uptime++;
}

void TIM3_IRQHandler(void)
{
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        if (++timer_loop_cnt_100hz * 100 >= 1000)
        {
            timer_loop_cnt_100hz  = 0;
            timer_loop_flag_100hz = true;
        }
        if (++timer_loop_cnt_50hz * 50 >= 1000)
        {
            timer_loop_cnt_50hz  = 0;
            timer_loop_flag_50hz = true;
        }
        if (++timer_loop_cnt_20hz * 20 >= 1000)
        {
            timer_loop_cnt_20hz  = 0;
            timer_loop_flag_20hz = true;
        }
        if (++timer_loop_cnt_10hz * 10 >= 1000)
        {
            timer_loop_cnt_10hz  = 0;
            timer_loop_flag_10hz = true;
        }
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
    {
        USART_SendData(USART1, USART_ReadBuffer(&USART_RingBufferTxStructure));
        if (USART_CountBuffer(&USART_RingBufferTxStructure) == 0)
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
    else if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        USART_WriteBuffer(&USART_RingBufferRxStructure,
                         (u8)USART_ReceiveData(USART1));
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
