/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_driver_usart.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.15
Description: define the serial port operation
Others:      none
Function List:
             1. extern void usart_Init(u32 bound);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_USART_H__
#define __STM32F10X_DRIVER_USART_H__

#include "stm32f10x.h"

#define USART_REC_LEN 200               /*定义最大接收字节数*/

extern u8 USART_RX_BUF[USART_REC_LEN];  /*接收缓冲，最大USART_REC_LEN个字节，最后一个字节为换行符*/
extern u16 USART_RX_STA;                /*接收状态标记*/

/*USART receiver buffer*/
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

extern UartBuf UartTxbuf;  /*环形发送队列*/
extern UartBuf UartRxbuf;  /*环形接收队列*/

extern void usart_Init(u32 bound);      /*USART_Init()在库函数中有*/

extern void USART_ClearBuf(UartBuf* RingBuf);
extern void USART_SendOneChar(unsigned char dat);
extern uint8_t USART_SendOneCharReturn(unsigned char dat);

extern uint8_t USART_ReadBuf(UartBuf* RingBuf);
extern uint16_t USART_CountBuf(UartBuf* RingBuf);
extern void USART_WriteBuf(UartBuf* RingBuf, uint8_t dat);
extern void USART_SendBuf(uint8_t* dat, uint8_t len);

#endif
