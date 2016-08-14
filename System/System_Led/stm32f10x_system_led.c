/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_system_led.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.14
Description: implement the led operation function
Others:      none
Function List:
             1. void led_init(void); 
             2. void led_test(void);
History:     none
*******************************************************************************/

#include "stm32f10x_system_led.h"

void led_init(void)
{
    RCC->APB2ENR |= 1 << 2;      /*Enable PORTA clock*/
    RCC->APB2ENR |= 1 << 3;      /*Enable PORTB clock*/

    RCC->APB2ENR |= 1 << 0;      /*Enable reset clock*/
    GPIOB->CRL   &= 0XFFFF0F0F;  /*PB1,3 push-pull output*/
    GPIOB->CRL   |= 0X00003030;
    GPIOB->ODR   |= 5 << 1;      /*PB1,3 pull-up*/
  
    GPIOA->CRH   &= 0XFFFF0FF0;  /*PA8,11 push-pull output*/
    GPIOA->CRH   |= 0X00003003;
    GPIOA->ODR   |= 9 << 0;      /*PA8,11 pull-up*/
  
    AFIO->MAPR   |= 2 << 24;
    LedA_off;
    LedB_off;
    LedC_off;
    LedD_off;
}

void led_test(void)
{
    LedA_on;
    LedB_on;
    LedC_on;
    LedD_on;
}
