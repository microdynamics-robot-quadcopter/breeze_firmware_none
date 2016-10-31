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

#include "stm32f10x_driver_delay.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"

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
//static u8 g_fac_us = 0;

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
//static u16 g_fac_ms = 0;

char SysClock;  /*申请存储系统时钟变量，单位MHz*/

/*不能在这里执行所有外设复位!否则至少引起串口不工作*/
/*把所有时钟寄存器复位*/
void MYRCC_DeInit(void)
{
    RCC->APB1RSTR = 0x00000000;  /*复位结束*/
    RCC->APB2RSTR = 0x00000000;
    RCC->AHBENR   = 0x00000014;  /*睡眠模式闪存和SRAM时钟使能.其他关闭*/
    RCC->APB2ENR  = 0x00000000;  /*外设时钟关闭*/
    RCC->APB1ENR  = 0x00000000;
    RCC->CR       |= 0x00000001; /*使能内部高速时钟HSION*/
    RCC->CFGR     &= 0xF8FF0000; /*复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]*/
    RCC->CR       &= 0xFEF6FFFF; /*复位HSEON,CSSON,PLLON*/
    RCC->CR       &= 0xFFFBFFFF; /*复位HSEBYP*/
    RCC->CFGR     &= 0xFF80FFFF; /*复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE*/
    RCC->CIR      = 0x00000000;  /*关闭所有中断*/
}

/********************************************
           使用外部晶体作为系统时钟源
功能：
1.使用外部HSE时钟8M作为PLL输入
2.PLL倍频系数PLLMUL<=9(实际到达16时，还能正常倍频外部时钟)
3.输入参数：PLLMUL，PLL倍频系数
4.备注：官方手册上说使用HSE作为系统时钟源时，最高可倍频到72MHz，但是实际可以倍频到128M系统还算稳定
********************************************/
/*系统时钟初始化函数*/
/*PLL:选择的倍频数，从2开始，最大值为16*/
/*时钟源为外部晶振*/
/*备注：当机身焊接了8M晶振时，就只能使用外部8M晶振作为时钟源*/
/*       用内部的HSI不好使，我反正没调出来，看各位有啥办法没*/
char SystemClock_HSE(u8 PLL)
{
    unsigned char temp = 0;
    MYRCC_DeInit();           /*复位并配置向量表*/
    RCC->CR |= 1<<16;         /*外部高速时钟使能HSEON*/
    while(!(RCC->CR >> 17));  /*等待外部时钟就绪*/
    RCC->CFGR = 0X00000400;   /*APB1=DIV2;APB2=DIV1;AHB=DIV1;*/
    PLL -= 2;                 /*抵消2个单位*/
    RCC->CFGR |= PLL<<18;     /*设置PLL值 2~16*/
    RCC->CFGR |= 1<<16;	      /*PLLSRC ON*/
    FLASH->ACR|= 0x32;	      /*FLASH 2个延时周期*/
    RCC->CR   |= 0x01000000;  /*PLLON*/
    while(!(RCC->CR >> 25));  /*等待PLL锁定*/
    RCC->CFGR |= 0x00000002;  /*PLL作为系统时钟*/
    while(temp != 0x02)       /*等待PLL作为系统时钟设置成功*/
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






/*由于使用外部时钟，利用SysTick定时中断来产生精确延时，所以不再使用以下函数*/
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
//void delay_init(void)
//{
//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  /*Select external clock HCLK/8*/
//	g_fac_us = SystemCoreClock / 8000000;                  /*Clock time's 1/8*/
//	g_fac_ms = (u16)g_fac_us * 1000;
//}

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
//void delay_us(u32 nus)
//{
//	u32 temp;
//	SysTick->LOAD = nus * g_fac_us;
//	SysTick->VAL  = 0x00;                        /*Clear timer*/
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /*Start countdown*/

//	do
//	{
//        temp = SysTick->CTRL;
//	}
//	while ((temp & 0x01) && !(temp & (1<<16)));  /*Wait time to arrive*/

//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /*Disable timer*/
//	SysTick->VAL  = 0x00;                        /*Clear timer*/
//}

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
//void delay_ms(u16 nms)
//{
//	u32 temp;
//	SysTick->LOAD = (u32)nms * g_fac_ms;         /*Load time, SysTick->LOAD is 24bit*/
//	SysTick->VAL  = 0x00;                        /*Clear timer*/
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    /*Start countdown*/

//	do
//	{
//		temp = SysTick->CTRL;
//	}
//	while ((temp & 0x01) && !(temp & (1<<16)));  /*Wait time to arrive*/

//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /*Disable timer*/
//	SysTick->VAL  = 0x00;                        /*Clear timer*/
//}
