/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_usart.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.15
Description: define the serial port operation
Others:      none
Function List:
             1. void usart_Init(u32 bound);
             2. void USART_ClearBuf(UartBuf* RingBuf);
             3. void USART_SendOneBytes(unsigned char dat);
             4. uint8_t USART_SendOneBytesReturn(unsigned char dat);
             5. uint8_t USART_ReadBuf(UartBuf* RingBuf);
             6. uint16_t USART_CountBuf(UartBuf* RingBuf);
             7. void USART_WriteBuf(UartBuf* RingBuf, uint8_t dat);
             8. void USART_SendBuf(uint8_t* dat, uint8_t len);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.06  modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_USART_H__
#define __STM32F10X_DRIVER_USART_H__

#include "stm32f10x.h"

#define USART_REC_LEN 200               /* The maximum number of bytes received */

extern u8 USART_RX_BUF[USART_REC_LEN];  /* Receive buffer, the last char is newline */
extern u16 USART_RX_STA;                /* The flag of receiving status */

/* USART receiver buffer */
#define RX_BUFFER_SIZE   128
#define TX_BUFFER_SIZE   128

extern unsigned char rx_buffer[RX_BUFFER_SIZE];
extern unsigned char tx_buffer[TX_BUFFER_SIZE];

typedef struct
{
    uint16_t volatile Wd_Indx;
    uint16_t volatile Rd_Indx;
    uint16_t Mask;
    uint8_t* pbuf;
}UartBuf;

extern UartBuf UartTxbuf;  /* Ring-send queue */
extern UartBuf UartRxbuf;  /* Ring-receive queue */

/* USART_Init() exists in the official library */
extern void usart_Init(u32 bound);      

extern void USART_ClearBuf(UartBuf* RingBuf);
extern void USART_SendOneBytes(unsigned char dat);
extern uint8_t USART_SendOneBytesReturn(unsigned char dat);

extern uint8_t USART_ReadBuf(UartBuf* RingBuf);
extern uint16_t USART_CountBuf(UartBuf* RingBuf);
extern void USART_WriteBuf(UartBuf* RingBuf, uint8_t dat);
extern void USART_SendBuf(uint8_t* dat, uint8_t len);

#endif
