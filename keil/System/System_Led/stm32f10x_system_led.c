/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_system_led.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.14
Description: implement the led operation function
Others:      none
Function List:
             1. extern void led_Init(void);
             2. extern void led_test(int flag);
History:     none
*******************************************************************************/

#include "stm32f10x_system_led.h"

void led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*硬件电路要求关闭JATG，千万不能将SWD也关闭，否则芯片作废!!!*/
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

//    RCC->APB2ENR |= 1<<2;        /*Enable PORTA clock*/
//    RCC->APB2ENR |= 1<<3;        /*Enable PORTB clock*/

//    RCC->APB2ENR |= 1<<0;        /*Enable reset clock*/
//    GPIOB->CRL   &= 0XFFFF0F0F;  /*PB1,3 push-pull output*/
//    GPIOB->CRL   |= 0X00003030;
//    GPIOB->ODR   |= 5<<1;        /*PB1,3 pull-up*/
//  
//    GPIOA->CRH   &= 0XFFFF0FF0;  /*PA8,11 push-pull output*/
//    GPIOA->CRH   |= 0X00003003;
//    GPIOA->ODR   |= 9<<0;        /*PA8,11 pull-up*/
//  
//    AFIO->MAPR   |= 2<<24; //关闭JATG,千万不能将SWD也关闭，否则芯片作废!!!

    LedA_Off;
    LedB_Off;
    LedC_Off;
    LedD_Off;
}

void led_test(int flag)
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
