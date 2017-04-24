/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_io.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.03
Description: Define the bitband operation
Others:      none
Function List:
             none
History:
<author>    <date>        <desc>
maksyuki    2016.11.29    Modify the module
myyerrol    2017.04.23    Format the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_SYS_H__
#define __STM32F10X_DRIVER_SYS_H__

#include "stm32f10x.h"

// Bitband operation.
// IO interfaces operation.
#define BIT_BAND(addr, bit_nums) ((addr & 0XF0000000) + 0X2000000 + \
                                 ((addr & 0XFFFFF) << 5) + (bit_nums << 2))
#define MEM_ADDR(addr)          *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bit_nums)   MEM_ADDR(BIT_BAND(addr, bit_nums))

// IO interfaces address mapping.
#define GPIOA_ODR_ADDR           (GPIOA_BASE + 12)
#define GPIOB_ODR_ADDR           (GPIOB_BASE + 12)
#define GPIOC_ODR_ADDR           (GPIOC_BASE + 12)
#define GPIOD_ODR_ADDR           (GPIOD_BASE + 12)
#define GPIOE_ODR_ADDR           (GPIOE_BASE + 12)
#define GPIOF_ODR_ADDR           (GPIOF_BASE + 12)
#define GPIOG_ODR_ADDR           (GPIOG_BASE + 12)

#define GPIOA_IDR_ADDR           (GPIOA_BASE + 8)
#define GPIOB_IDR_ADDR           (GPIOB_BASE + 8)
#define GPIOC_IDR_ADDR           (GPIOC_BASE + 8)
#define GPIOD_IDR_ADDR           (GPIOD_BASE + 8)
#define GPIOE_IDR_ADDR           (GPIOE_BASE + 8)
#define GPIOF_IDR_ADDR           (GPIOF_BASE + 8)
#define GPIOG_IDR_ADDR           (GPIOG_BASE + 8)

#define PA_IN(n)                  BIT_ADDR(GPIOA_IDR_ADDR, n)
#define PA_OUT(n)                 BIT_ADDR(GPIOA_ODR_ADDR, n)
#define PB_IN(n)                  BIT_ADDR(GPIOB_IDR_ADDR, n)
#define PB_OUT(n)                 BIT_ADDR(GPIOB_ODR_ADDR, n)
#define PC_IN(n)                  BIT_ADDR(GPIOC_IDR_ADDR, n)
#define PC_OUT(n)                 BIT_ADDR(GPIOC_ODR_ADDR, n)
#define PD_IN(n)                  BIT_ADDR(GPIOD_IDR_ADDR, n)
#define PD_OUT(n)                 BIT_ADDR(GPIOD_ODR_ADDR, n)
#define PE_IN(n)                  BIT_ADDR(GPIOE_IDR_ADDR, n)
#define PE_OUT(n)                 BIT_ADDR(GPIOE_ODR_ADDR, n)
#define PF_IN(n)                  BIT_ADDR(GPIOF_IDR_ADDR, n)
#define PF_OUT(n)                 BIT_ADDR(GPIOF_ODR_ADDR, n)
#define PG_IN(n)                  BIT_ADDR(GPIOG_IDR_ADDR, n)
#define PG_OUT(n)                 BIT_ADDR(GPIOG_ODR_ADDR, n)

#endif
