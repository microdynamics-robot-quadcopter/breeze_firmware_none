/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_delay.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.02
Description: implement the time delay function
Others:      none
Function List:
             1. void delay_init(void);
             2. void delay_us(u32 nus);
             3. void delay_ms(u16 nms);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.15  modify the module
*******************************************************************************/

#include "stm32f10x_driver_delay.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"

//static u8 g_fac_us = 0;
//static u16 g_fac_ms = 0;

char SysClock;

/* Reset all registers */
/* Can not reset all the peripheral, otherwise the serial port doesn't work */
void MYRCC_DeInit(void)
{
    RCC->APB1RSTR = 0x00000000; /* Reset is end */
    RCC->APB2RSTR = 0x00000000;
    RCC->AHBENR   = 0x00000014; /* Enable sleep mode flash and SRAM clock, disable others */
    RCC->APB2ENR  = 0x00000000; /* Disable peripheral clock */
    RCC->APB1ENR  = 0x00000000;
    RCC->CR      |= 0x00000001; /* Enable internal high-speed clock HSION */
    RCC->CFGR    &= 0xF8FF0000; /* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0], MCO[2:0] */
    RCC->CR      &= 0xFEF6FFFF; /* Reset HSEON, CSSON, PLLON */
    RCC->CR      &= 0xFFFBFFFF; /* Reset HSEBYP */
    RCC->CFGR    &= 0xFF80FFFF; /* Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE */
    RCC->CIR      = 0x00000000; /* Disable all the peripheral */
}

/* PLL: 2~16 */
char SystemClock_HSE(u8 PLL)
{
    unsigned char temp = 0;
    MYRCC_DeInit();           /* Reset and configure vector table */
    RCC->CR |= 1<<16;         /* Enable external high-speed clock HSEON */
    while(!(RCC->CR >> 17));  /* Wait the external clock is prepared */
    RCC->CFGR = 0X00000400;   /* APB1=DIV2; APB2=DIV1; AHB=DIV1; */
    PLL -= 2;
    RCC->CFGR |= PLL<<18;     /* Set PLL 2~16 */
    RCC->CFGR |= 1<<16;       /* PLLSRC ON */
    FLASH->ACR|= 0x32;        /* FLASH 2 delay period */
    RCC->CR   |= 0x01000000;  /* PLLON */
    while(!(RCC->CR >> 25));  /* Wait PLL is locked */
    RCC->CFGR |= 0x00000002;  /* PLL is as system clock */
    while(temp != 0x02)       /* Wait the configuration is completed which PLL is as system clock */
    {
        temp = RCC->CFGR >> 2;
        temp &= 0x03;
    }
    SysClock =(PLL + 2) * 8;
    return SysClock;
}

void delay_us(u32 nus)
{
    u32 t0 = micros();
    while(micros() - t0 < nus);
}

void delay_ms(u16 nms)
{
    u32 t0 = micros();
    while(micros() - t0 < nms * 1000);
}

/* Don't use this functions */
//void delay_init(void)
//{
//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  /* Select external clock HCLK/8 */
//	g_fac_us = SystemCoreClock / 8000000;                  /* Clock time's 1/8 */
//	g_fac_ms = (u16)g_fac_us * 1000;
//}

//void delay_us(u32 nus)
//{
//	u32 temp;
//	SysTick->LOAD = nus * g_fac_us;
//	SysTick->VAL  = 0x00;                        /* Clear timer */
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /* Start countdown */

//	do
//	{
//        temp = SysTick->CTRL;
//	}
//	while ((temp & 0x01) && !(temp & (1<<16)));  /* Wait time to arrive */

//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /* Disable timer */
//	SysTick->VAL  = 0x00;                        /* Clear timer */
//}

//void delay_ms(u16 nms)
//{
//	u32 temp;
//	SysTick->LOAD = (u32)nms * g_fac_ms;         /* Load time, SysTick->LOAD is 24bit */
//	SysTick->VAL  = 0x00;                        /* Clear timer */
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /* Start countdown */

//	do
//	{
//		temp = SysTick->CTRL;
//	}
//	while ((temp & 0x01) && !(temp & (1<<16)));  /* Wait time to arrive */

//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /* Disable timer */
//	SysTick->VAL  = 0x00;                        /* Clear timer */
//}
