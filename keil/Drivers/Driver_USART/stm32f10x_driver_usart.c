/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_driver_usart.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.15
Description: implement the serial port operation
Others:      none
Function List:
             1. extern void usart_Init(u32 bound);
History:     none
*******************************************************************************/

#include "stm32f10x_driver_usart.h"
#include "stdio.h"

/*加入以下代码支持printf函数*/
#if 1
#pragma import(__use_no_semihosting)

/*标准库需要的支持函数*/
struct __FILE
{
    int handle;
};

FILE __stdout;

/*定义_sys_exit()来避免使用半主机模式*/
_sys_exit(int x)
{
    x = x;
}

/*重定义fputc函数*/
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0x40) == 0);  /*循环发送直到发送完成*/
    USART1->DR = (u8)ch;
    return ch;
}

#endif

/*注意读取USARTx->SR能避免莫名其妙的错误*/
u8 USART_RX_BUF[USART_REC_LEN];  /*接收缓冲，最大USART_REC_LEN个字节*/

/*接收状态*/
/*bit15     接收完成标志*/
/*bit14     接收到0x0d*/
/*bit13~0   接收到的有效字节数目*/
u16 USART_RX_STA = 0;            /*接收状态标记*/

void usart_Init(u32 bound)
{
    /*GPIO端口设置*/
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /*Reset the USART1*/
    USART_DeInit(USART1);

    /*USART1_TX  GPIOA.9*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*USART1_RX  GPIOA.10*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*USART1 NVIC配置*/
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;              /*指定USART1通道中断*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                        /*抢占优先级3*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;                        /*子优先级3*/
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;                   /*IRQ通道使能*/
    NVIC_Init(&NVIC_InitStructure);                                                  /*根据指定的参数初始化VIC寄存器*/

    /*USART 初始化设置*/
    USART_InitStructure.USART_BaudRate            = bound;                           /*串口波特率*/
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;             /*字长为8位数据格式*/
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;                /*一个停止位*/
    USART_InitStructure.USART_Parity              = USART_Parity_No;                 /*无奇偶校验位*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  /*无硬件数据流控制*/
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;   /*收发模式*/

    USART_Init(USART1, &USART_InitStructure);                                        /*初始化串口1*/
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                   /*开启串口接收中断*/
    USART_Cmd(USART1, ENABLE);                                                       /*使能串口1*/
}

/*串口1中断服务程序*/
void USART1_IRQHandler(void)
{
    u8 res;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  /*接收中断(接收到的数据必须是0x0d 0x0a结尾)*/
    {
        res = USART_ReceiveData(USART1);                    /*读取接收到的数据*/
        if ((USART_RX_STA & 0x8000) == 0)                   /*接收未完成*/
        {
            if (USART_RX_STA & 0x4000)                      /*接收到了0x0d*/
            {
                if (res != 0x0A)
                {
                    USART_RX_STA = 0;                       /*接收错误, 重新开始*/
                }
                else
                {
                    USART_RX_STA |= 0x8000;                 /*接收完成了*/
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
                        USART_RX_STA = 0;                   /*接收数据错误,重新开始接收*/
                    }
                }
            }
        }
    }
    else if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        USART_SendData(USART1, USART_ReadBuf(&UartTxbuf));     /*环形队列数据缓存发送*/
        if (USART_CountBuf(&UartTxbuf) == 0)
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);     /*假如缓冲空了，就关闭串口发送中断*/
        }
    }
}

void USART_SendOneChar(unsigned char dat)
{
    USART_WriteBuf(&UartTxbuf, dat);               /*将待发送数据放在环形缓冲队列中*/
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  /*启动发送中断开始发送缓冲中的数据*/
}

uint8_t USART_SendOneCharReturn(unsigned char dat)
{
    USART_WriteBuf(&UartTxbuf, dat);               /*将待发送数据放在环形缓冲队列中*/
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  /*启动发送中断开始发送缓冲中的数据*/
    return dat;
}

/*环形队列结构体实例化两个变量*/
UartBuf UartTxbuf;  /*环形发送队列*/
UartBuf UartRxbuf;  /*环形接收队列*/

unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char tx_buffer[TX_BUFFER_SIZE];

/*读取环形数据队列中的一个字节*/
uint8_t USART_ReadBuf(UartBuf* RingBuf)
{
    uint8_t temp;
    temp = RingBuf->pbuf[RingBuf->Rd_Indx&RingBuf->Mask];  /*数据长度掩码很重要，这是决定数据环形的关键*/
    RingBuf->Rd_Indx++;                                    /*读取完成一次读指针加1，为下一次读取做准备*/
    return temp;
}

/*将一个字节写入一个环形队列中*/
void USART_WriteBuf(UartBuf* RingBuf, uint8_t dat)
{
    RingBuf->pbuf[RingBuf->Wd_Indx&RingBuf->Mask] = dat;    /*数据长度掩码很重要，这是决定数据环形的关键*/
    RingBuf->Wd_Indx++;                                     /*写完一次写指针加1，为下一次写入做准备*/
}

/*环形数据区的可用字节长度，当写指针写完一圈，追上了读指针*/
/*那么证明数据写满了，此时应该增加缓冲区长度，或者缩短外围数据处理时间*/
uint16_t USART_CountBuf(UartBuf* RingBuf)
{
    return (RingBuf->Wd_Indx - RingBuf->Rd_Indx) & RingBuf->Mask;  /*数据长度掩码很重要，这是决定数据环形的关键*/
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
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);   /*启动发送中断开始发送缓冲中的数据*/
}
