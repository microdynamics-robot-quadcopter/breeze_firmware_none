#ifndef __STM32F10x_DRIVER_IO_H__
#define __STM32F10x_DRIVER_IO_H__

#include "stm32f10x.h"

#define BITBAND(addr, bitnum)   ((addr & 0xF0000000) + 0x2000000 + \
                                ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr)          *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_ADDR    (GPIOA_BASE + 12) //0x4001080C
#define GPIOB_ODR_ADDR    (GPIOB_BASE + 12) //0x40010C0C
#define GPIOC_ODR_ADDR    (GPIOC_BASE + 12) //0x4001100C
#define GPIOD_ODR_ADDR    (GPIOD_BASE + 12) //0x4001140C
#define GPIOE_ODR_ADDR    (GPIOE_BASE + 12) //0x4001180C
#define GPIOF_ODR_ADDR    (GPIOF_BASE + 12) //0x40011A0C
#define GPIOG_ODR_ADDR    (GPIOG_BASE + 12) //0x40011E0C

#define GPIOA_IDR_ADDR    (GPIOA_BASE + 8)  //0x40010808
#define GPIOB_IDR_ADDR    (GPIOB_BASE + 8)  //0x40010C08
#define GPIOC_IDR_ADDR    (GPIOC_BASE + 8)  //0x40011008
#define GPIOD_IDR_ADDR    (GPIOD_BASE + 8)  //0x40011408
#define GPIOE_IDR_ADDR    (GPIOE_BASE + 8)  //0x40011808
#define GPIOF_IDR_ADDR    (GPIOF_BASE + 8)  //0x40011A08
#define GPIOG_IDR_ADDR    (GPIOG_BASE + 8)  //0x40011E08

#define PA_OUT(n)   BIT_ADDR(GPIOA_ODR_ADDR, n)
#define PA_IN(n)    BIT_ADDR(GPIOA_IDR_ADDR, n)

#define PB_OUT(n)   BIT_ADDR(GPIOB_ODR_ADDR, n)
#define PB_IN(n)    BIT_ADDR(GPIOB_IDR_ADDR, n)

#define PC_OUT(n)   BIT_ADDR(GPIOC_ODR_ADDR, n)
#define PC_IN(n)    BIT_ADDR(GPIOC_IDR_ADDR, n)

#define PD_OUT(n)   BIT_ADDR(GPIOD_ODR_ADDR, n)
#define PD_IN(n)    BIT_ADDR(GPIOD_IDR_ADDR, n)

#define PE_OUT(n)   BIT_ADDR(GPIOE_ODR_ADDR, n)
#define PE_IN(n)    BIT_ADDR(GPIOE_IDR_ADDR, n)

#define PF_OUT(n)   BIT_ADDR(GPIOF_ODR_ADDR, n)
#define PF_IN(n)    BIT_ADDR(GPIOF_IDR_ADDR, n)

#define PG_OUT(n)   BIT_ADDR(GPIOG_ODR_ADDR, n)
#define PG_IN(n)    BIT_ADDR(GPIOG_IDR_ADDR, n)

#endif
