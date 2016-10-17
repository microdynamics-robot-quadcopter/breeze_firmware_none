/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_driver_spi.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.14
Description: declare the spi bus operation function
Others:      none
Function List:
             1. extern void SPI_INIT(void);
             2. extern u8 SPI_RW(u8 data);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_SPI_H__
#define __STM32F10X_DRIVER_SPI_H__

#include "stm32f10x.h"
/*
6.2.4G:	CE:   PA15
        CSN:  PA4
        SCK:  PA5
        MOSI: PA7
        MISO: PA6
        IRQ:  PA8
*/
#define SPI_CE_H()   GPIO_SetBits(GPIOA, GPIO_Pin_12)
#define SPI_CE_L()   GPIO_ResetBits(GPIOA, GPIO_Pin_12)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)

extern void SPI_INIT(void);
extern u8 SPI_RW(u8 data);

#endif
