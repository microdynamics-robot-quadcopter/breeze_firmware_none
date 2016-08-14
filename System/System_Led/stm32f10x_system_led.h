/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_system_led.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.14
Description: declare the time led function
Others:      none
Function List:
             1. void led_init(void); 
             2. void led_test(void);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_SYSTEM_LED__
#define __STM32F10X_SYSTEM_LED__

#include "stm32f10x.h"

#define LedA_on    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LedA_off   GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define LedB_on    GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LedB_off   GPIO_ResetBits(GPIOA, GPIO_Pin_8)

#define LedC_on    GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define LedC_off   GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define LedD_on    GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LedD_off   GPIO_ResetBits(GPIOB, GPIO_Pin_3)

#define LEDB_troggle GPIO_WriteBit(GPIOA,GPIO_Pin_8, !GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_8))
#define LEDC_troggle GPIO_WriteBit(GPIOB,GPIO_Pin_1, !GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_1))
#define LEDA_troggle GPIO_WriteBit(GPIOA,GPIO_Pin_11, !GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_11))

extern void led_init(void);    /*Initialize led */
extern void led_test(void);

#endif
