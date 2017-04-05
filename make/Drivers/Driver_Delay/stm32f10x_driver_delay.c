#include "stm32f10x_driver_delay.h"

static u8  g_factor_us = 0;
static u16 g_factor_ms = 0;

void Delay_Init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    g_factor_us = SystemCoreClock / 8000000;
    g_factor_ms = (u16)g_factor_us * 1000;
}

void Delay_TimeMs(u16 n_ms)
{
    u32 temp;

    SysTick->LOAD  = (u32)n_ms * g_factor_ms;
    SysTick->VAL   = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    do
    {
        temp = SysTick->CTRL;
    }
    while ((temp & 0x01) && !(temp & (1 << 16)));

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL   = 0x00;
}

void Delay_TimeUs(u32 n_us)
{
    u32 temp;

    SysTick->LOAD  = n_us * g_factor_us;
    SysTick->VAL   = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    do
    {
        temp = SysTick->CTRL;
    }
    while ((temp & 0x01) && !(temp & (1 << 16)));

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL   = 0x00;
}
