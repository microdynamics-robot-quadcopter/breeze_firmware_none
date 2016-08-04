/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_driver_sys.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.3
Description: define the bitband operation
Others:      none
Function List:
             none
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_SYS_H__
#define __STM32F10X_DRIVER_SYS_H__	
#include "stm32f10x.h"

/*Bitband operation*/
/*IO interfaces operation*/
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2)) 
#define MEM_ADDR(addr) *((volatile unsigned long*)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

/*IO interfaces address mapping*/
#define GPIOA_ODR_Addr    (GPIOA_BASE + 12) /*0x4001080C*/ 
#define GPIOB_ODR_Addr    (GPIOB_BASE + 12) /*0x40010C0C*/ 
#define GPIOC_ODR_Addr    (GPIOC_BASE + 12) /*0x4001100C*/ 
#define GPIOD_ODR_Addr    (GPIOD_BASE + 12) /*0x4001140C*/ 
#define GPIOE_ODR_Addr    (GPIOE_BASE + 12) /*0x4001180C*/
#define GPIOF_ODR_Addr    (GPIOF_BASE + 12) /*0x40011A0C*/  
#define GPIOG_ODR_Addr    (GPIOG_BASE + 12) /*0x40011E0C*/

#define GPIOA_IDR_Addr    (GPIOA_BASE + 8)  /*0x40010808*/
#define GPIOB_IDR_Addr    (GPIOB_BASE + 8)  /*0x40010C08*/ 
#define GPIOC_IDR_Addr    (GPIOC_BASE + 8)  /*0x40011008*/ 
#define GPIOD_IDR_Addr    (GPIOD_BASE + 8)  /*0x40011408*/ 
#define GPIOE_IDR_Addr    (GPIOE_BASE + 8)  /*0x40011808*/
#define GPIOF_IDR_Addr    (GPIOF_BASE + 8)  /*0x40011A08*/
#define GPIOG_IDR_Addr    (GPIOG_BASE + 8)  /*0x40011E08*/ 
 
/*Only for one IO interface!*/
/*Make sure the value of n is less than 16!*/
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr, n)  /*Output*/ 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr, n)  /*Input*/ 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr, n)  /*Output*/
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr, n)  /*Input*/ 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr, n)  /*Output*/
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr, n)  /*Input*/ 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr, n)  /*Output*/ 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr, n)  /*Input*/ 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr, n)  /*Output*/
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr, n)  /*Input*/ 

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr, n)  /*Output*/
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr, n)  /*Input*/ 

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr, n)  /*Output*/
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr, n)  /*Input*/ 

#endif
