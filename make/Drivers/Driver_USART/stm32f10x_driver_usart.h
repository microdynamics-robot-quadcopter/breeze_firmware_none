#ifndef __STM32F10x_DRIVER_USART_H
#define __STM32F10x_DRIVER_USART_H

#include "stm32f10x.h"

#define USART_RX_LENGTH 200
#define USART_RX_ENABLE 1

extern u8  USART_RX_BUFFER[USART_RX_LENGTH];
extern u16 USART_RX_STATE;

void UASRT_Init(u32 baud_rate);

#endif
