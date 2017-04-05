#ifndef __STM32F10x_DRIVER_DELAY_H__
#define __STM32F10x_DRIVER_DELAY_H__

#include "stm32f10x.h"

void Delay_Init(void);
void Delay_TimeMs(u16 n_ms);
void Delay_TimeUs(u32 n_us);

#endif
