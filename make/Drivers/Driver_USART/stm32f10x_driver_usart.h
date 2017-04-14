/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_usart.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.15
Description: Define the serial port operation
Others:      none
Function List:
             1. void USART_ClearBuffer(USART_RingBuffer *ring_buffer);
             2. void USART_InitUSART(u32 baud_rate);
             3. void USART_InitUSART1(u32 baud_rate);
             4. void USART_SendBuffer(u8 *bytes, u8 length);
             5. void USART_SendByte(u8 byte);
             6. void USART_WriteBuffer(USART_RingBuffer *ring_buffer, u8 byte);
             7. u8   USART_ReadBuffer(USART_RingBuffer *ring_buffer);
             8. u16  USART_CountBuffer(USART_RingBuffer *ring_buffer);
History:
<author>    <date>        <desc>
maksyuki    2016.12.06    Modify the module
myyerrol    2017.04.14    Format the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_USART_H__
#define __STM32F10X_DRIVER_USART_H__

#include "stm32f10x.h"

#define BUFFER_SIZE 128

extern u8 ring_buffer_rx[BUFFER_SIZE];
extern u8 ring_buffer_tx[BUFFER_SIZE];

typedef struct
{
    u8  *buffer;
    u16  mask;
    vu16 index_rd;
    vu16 index_wt;
} USART_RingBuffer;

extern USART_RingBuffer USART_RingBufferRxStructure;
extern USART_RingBuffer USART_RingBufferTxStructure;

extern void USART_ClearBuffer(USART_RingBuffer *ring_buffer);
extern void USART_InitUSART(u32 baud_rate);
extern void USART_InitUSART1(u32 baud_rate);
extern void USART_SendBuffer(u8 *bytes, u8 length);
extern void USART_SendByte(u8 byte);
extern void USART_WriteBuffer(USART_RingBuffer *ring_buffer, u8 byte);
extern u8   USART_ReadBuffer(USART_RingBuffer *ring_buffer);
extern u16  USART_CountBuffer(USART_RingBuffer *ring_buffer);

#endif
