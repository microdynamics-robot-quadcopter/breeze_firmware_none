/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_driver_spi.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.14
Description: implement the spi bus operation function
Others:      none
Function List:
             1. extern void SPI_INIT(void);
             2. extern u8 SPI_RW(u8 data);
History:     none
*******************************************************************************/

#include "stm32f10x_driver_spi.h"

extern void SPI_INIT(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /*配置 SPI_NRF_SPI的 SCK,MISO,MOSI引脚，GPIOA^5,GPIOA^6,GPIOA^7*/ 
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;  /*复用功能*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*配置SPI_NRF_SPI的CE引脚，和SPI_NRF_SPI的CSN引脚*/
    /*NRF_CE--PA12*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*NRF_CSN--PA4*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_CSN_H();

    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex; /*双线全双工*/
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;                 /*主模式*/
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;                 /*数据大小8位*/
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;                    /*时钟极性，空闲时为低*/
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;                  /*第1个边沿有效，上升沿为采样时刻*/
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;                    /*NSS信号由软件产生*/
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;         /*8分频，9MHz*/
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;                /*高位在前*/
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    /* Enable SPI1 */
    SPI_Cmd(SPI1, ENABLE);
}

extern u8 SPI_RW(u8 data)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI1); 
}
