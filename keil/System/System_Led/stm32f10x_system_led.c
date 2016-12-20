/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_system_led.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: implement the led function
Others:      none
Function List:
             1. void LED_Init(void);
             2. void LED_test(int flag);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.20  modify the module
*******************************************************************************/

#include "stm32f10x_system_led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* When hardware disables the JATG, don't disable the SWD meanwhile */
    /* Otherwise the chip will be damaged!!! */
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

//    RCC->APB2ENR |= 1<<2;       /* Enable PORTA clock */
//    RCC->APB2ENR |= 1<<3;       /* Enable PORTB clock */

//    RCC->APB2ENR |= 1<<0;       /* Enable reset clock */
//    GPIOB->CRL   &= 0XFFFF0F0F; /* PB1,3 push-pull output */
//    GPIOB->CRL   |= 0X00003030;
//    GPIOB->ODR   |= 5<<1;       /* PB1,3 pull-up */
//  
//    GPIOA->CRH   &= 0XFFFF0FF0; /* PA8,11 push-pull output */
//    GPIOA->CRH   |= 0X00003003;
//    GPIOA->ODR   |= 9<<0;       /* PA8,11 pull-up */
//  
//    AFIO->MAPR   |= 2<<24;      /* When hardware disables the JATG, don't disable the SWD meanwhile */
                                  /* Otherwise the chip will be damaged!!! */

    LedA_Off;
    LedB_Off;
    LedC_Off;
    LedD_Off;
}

void LED_test(int flag)
{
    if (flag == 1)
    {
        LedA_On;
        LedB_On;
        LedC_On;
        LedD_On;
    }
    else
    {
        LedA_Off;
        LedB_Off;
        LedC_Off;
        LedD_Off;
    }
}
