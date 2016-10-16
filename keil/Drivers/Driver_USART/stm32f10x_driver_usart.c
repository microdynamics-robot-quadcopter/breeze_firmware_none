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

#if EN_USART_RX                  /*若使能了接收标志*/
/*串口1中断服务程序*/
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
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn; /*指定USART1通道中断*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;           /*抢占优先级3*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;           /*子优先级3*/
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;      /*IRQ通道使能*/
    NVIC_Init(&NVIC_InitStructure);                                     /*根据指定的参数初始化VIC寄存器*/

    /*USART 初始化设置*/
    USART_InitStructure.USART_BaudRate            = bound;                           /*串口波特率*/
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;             /*字长为8位数据格式*/
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;                /*一个停止位*/
    USART_InitStructure.USART_Parity              = USART_Parity_No;                 /*无奇偶校验位*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  /*无硬件数据流控制*/
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;   /*收发模式*/

    USART_Init(USART1, &USART_InitStructure);       /*初始化串口1*/
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  /*开启串口接收中断*/
    USART_Cmd(USART1, ENABLE);                      /*使能串口1*/
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
                if (res != 0x0a)
                {
                    USART_RX_STA = 0;                       /*接收错误,重新开始*/
                }
                else
                {
                    USART_RX_STA |= 0x8000;                 /*接收完成了*/
                }
            }
            else
            {
                if (res == 0x0d)
                {
                    USART_RX_STA |= 0x4000;
                }
                else
                {
                    USART_RX_BUF[USART_RX_STA&0x3fff] = res;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                    {
                        USART_RX_STA = 0;                   /*接收数据错误,重新开始接收*/
                    }
                }
            }
        }
    }
}

#endif
