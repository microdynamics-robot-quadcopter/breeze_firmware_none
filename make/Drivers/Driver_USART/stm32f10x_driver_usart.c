#include <stdio.h>
#include "stm32f10x_driver_usart.h"

#if 1
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
}

FILE __stdout;
_sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0x40) == 0);
    USART1->DR = (u8)ch;
    return ch;
}
#endif

#if USART_RX_ENABLE
// Bit 15:   Receive complete flag
// Bit 14:   Receive 0x0A flag
// Bit 13~0: Receive data number
u8  USART_RX_BUFFER[USART_RX_LENGTH];
u16 USART_RX_STATE = 0;

void USART_Init(u32 baud_rate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,
                           ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate   = baud_rate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
                                           USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    u8 data;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART1);
        if (!(USART_RX_STATE & 0x8000))
        {
            if (USART_RX_STATE & 0x4000)
            {
                USART_RX_STATE |= 0x8000;
            }
            else
            {
                if (data == 0x0a)
                {
                    USART_RX_STATE |= 0x4000;
                }
                else
                {
                    USART_RX_BUFFER[USART_RX_STATE & 0x3FFF] = data;
                    USART_RX_STATE++;
                    if (USART_RX_STATE > (USART_RX_LENGTH - 1))
                    {
                        USART_RX_STATE = 0;
                    }
                }
            }
        }
    }
}
