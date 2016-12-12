/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_usart.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.15
Description: implement the serial port operation
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
   maksyuki  2016.12.12  modify the module
*******************************************************************************/

#include "stm32f10x_driver_usart.h"
#include "stdio.h"

/* Add the below code to support 'printf' function */
#if 1
#pragma import(__use_no_semihosting)

/* The support function is needed by standard library */
struct __FILE
{
    int handle;
};

FILE __stdout;

/* Define _sys_exit() to avoid to use semihosting */
_sys_exit(int x)
{
    x = x;
}

/* Redefine 'fputc' function */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0x40) == 0);  /* Cyclic send until complete */
    USART1->DR = (u8)ch;
    return ch;
}

#endif

/* Notice: read USARTx->SR can avoid bizarre error */
u8 USART_RX_BUF[USART_REC_LEN];

/* Receive status */
/* bit15     complete flag */
/* bit14     receive 0x0d */
/* bit13~0   receive the number of effective byte */
u16 USART_RX_STA = 0;

void usart_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* Reset the USART1 */
    USART_DeInit(USART1);

    /* USART1_TX  GPIOA.9 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1_RX  GPIOA.10 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 NVIC configuration */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;              /* USART1 channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                        /* Preemption priority level */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;                        /* SubPriority */
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;                   /* Enable IRQ channel */
    NVIC_Init(&NVIC_InitStructure);                                                  /* Initialize  NVIC */

    /* USART initialization settings */
    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;                /* One stop bit */
    USART_InitStructure.USART_Parity              = USART_Parity_No;                 /* No parity bit */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;   /* Send-receive mode */

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  /* Enable serial port interrupt to read data */
    USART_Cmd(USART1, ENABLE);                      /* Enable USART1 */

    UartTxbuf.Wd_Indx = 0;                          /* Initialize ring queue */
    UartTxbuf.Rd_Indx = 0;
    UartTxbuf.Mask    = TX_BUFFER_SIZE - 1;
    UartTxbuf.pbuf    = &tx_buffer[0];
}

/* Interrupt service routine */
void USART1_IRQHandler(void)
{
    u8 res;

    /* Receive interrupt(the data must end with [0x0d 0x0a] */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART1);                    /* Read received data */
        if ((USART_RX_STA & 0x8000) == 0)                   /* Receiving is not completed */
        {
            if (USART_RX_STA & 0x4000)                      /* Receive 0x0d */
            {
                if (res != 0x0A)
                {
                    USART_RX_STA = 0;                       /* Receiving error, restart */
                }
                else
                {
                    USART_RX_STA |= 0x8000;                 /* Receiving is completed */
                }
            }
            else
            {
                if (res == 0x0D)
                {
                    USART_RX_STA |= 0x4000;
                }
                else
                {
                    USART_RX_BUF[USART_RX_STA&0x3FFF] = res;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                    {
                        USART_RX_STA = 0;                   /* Data error, restart receiving */
                    }
                }
            }
        }
    }
    else if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        USART_SendData(USART1, USART_ReadBuf(&UartTxbuf));  /* Ring-send queue sends data with buffer */
        if (USART_CountBuf(&UartTxbuf) == 0)
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  /* If buffer is empty, disable serial port interrupt */
        }
    }
}

UartBuf UartTxbuf;
UartBuf UartRxbuf;

unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char tx_buffer[TX_BUFFER_SIZE];


void USART_SendOneBytes(unsigned char dat)
{
    USART_WriteBuf(&UartTxbuf, dat);               /* Place data into Ring-send queue */
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  /* Enable serial port interrupt to send data */
}

uint8_t USART_SendOneBytesReturn(unsigned char dat)
{
    USART_WriteBuf(&UartTxbuf, dat);               /* Place data into Ring-send queue */
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  /* Enable serial port interrupt to send data */
    return dat;
}

/* Read one byte in Ring-send queue */
uint8_t USART_ReadBuf(UartBuf* RingBuf)
{
    uint8_t temp;
    temp = RingBuf->pbuf[RingBuf->Rd_Indx&RingBuf->Mask];  /* Length mask is vital */
    RingBuf->Rd_Indx++;
    return temp;
}

/* Write one byte in Ring-send queue */
void USART_WriteBuf(UartBuf* RingBuf, uint8_t dat)
{
    RingBuf->pbuf[RingBuf->Wd_Indx&RingBuf->Mask] = dat;   /* Length mask is vital */
    RingBuf->Wd_Indx++;
}

uint16_t USART_CountBuf(UartBuf* RingBuf)
{
    return (RingBuf->Wd_Indx - RingBuf->Rd_Indx) & RingBuf->Mask;
}

void USART_ClearBuf(UartBuf* RingBuf)
{
    RingBuf->Rd_Indx = RingBuf->Wd_Indx;
}

void USART_SendBuf(uint8_t* dat, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        USART_WriteBuf(&UartTxbuf, *dat);
        dat++;
    }
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  /* Enable serial port interrupt to send data */
}
