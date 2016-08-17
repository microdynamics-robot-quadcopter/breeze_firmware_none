/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_driver_delay.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.2
Description: implement the time delay function
Others:      none
Function List:
             1. extern void delay_init(void);
             2. extern void delay_us(u32 nus);
             3. extern void delay_ms(u16 nms);
History:     none
*******************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_driver_delay.h"

/*******************************************************************************
* Global variable function:
*      the timing factor in microsecond precision
* Avalable value:
*      0      - initial value
*      Others - set value
* Call relationship:
*      only function delay_init(void) and delay_us(u32 nus)
*      in this modual can modify it
*******************************************************************************/
static u8 g_fac_us = 0;

/*******************************************************************************
* Global variable function:
*      the timing factor in millisecond precision
* Avalable value:
*      0      - initial value
*      Others - set value
* Call relationship:
*      only function delay_init(void) and delay_ms(u16 nms)
*      in this modual can modify it
*******************************************************************************/
static u16 g_fac_ms = 0;

/*******************************************************************************
Function:       void delay_init(void)
Description:    initializes the delay function
Calls:          void SysTick_CLKSourceConfig(void)
Called By:      int main(void)
Table Accessed: none
Table Updated:  none
Input:          none
Output:         none
Return:         none
Others:         none
*******************************************************************************/
void delay_init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  /*Select external clock HCLK/8*/
	g_fac_us = SystemCoreClock / 8000000;                  /*Clock time's 1/8*/
	g_fac_ms = (u16)g_fac_us * 1000;
}

/*******************************************************************************
Function:       void delay_us(u32 nus)
Description:    set time with microsecond precision
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          set value, the maximum value is [2^24/g_fac_us]
Output:         none
Return:         none
Others:         to use SysTick clock to delay time, an external
                8MHz crystal oscillator is required
*******************************************************************************/ 								   
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = nus * g_fac_us;
	SysTick->VAL  = 0x00;                        /*Clear timer*/
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /*Start countdown*/

	do
	{
        temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1<<16)));  /*Wait time to arrive*/

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /*Disable timer*/
	SysTick->VAL  = 0x00;                        /*Clear timer*/
}

/*******************************************************************************
Function:       void delay_ms(u16 nms)
Description:    set time with millisecond precision
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          set value, the maximum value is [0xffffff*8*1000/SYSCLK]
Output:         none
Return:         none
Others:         to use SysTick clock to delay time, an external 
                8MHz crystal oscillator is required, the maximum
                delay is 1864ms
*******************************************************************************/
void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms * g_fac_ms;         /*Load time, SysTick->LOAD is 24bit*/
	SysTick->VAL  = 0x00;                        /*Clear timer*/
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /*Start countdown*/

	do
	{
		temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1<<16)));  /*Wait time to arrive*/

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /*Disable timer*/
	SysTick->VAL  = 0x00;                        /*Clear timer*/
}
