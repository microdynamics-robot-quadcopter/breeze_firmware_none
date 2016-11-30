/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_spi.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.8.14
Description: declare the SPI bus operation function
Others:      none
Function List:
             1. void SPI_INIT(void);
             2. u8 SPI_RW(u8 data);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.11.30  modify the module
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
