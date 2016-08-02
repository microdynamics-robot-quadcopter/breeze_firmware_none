#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f10x.h"

extern void delay_init(void);
extern void delay_ms(u16 nms);
extern void delay_us(u32 nus);
#endif
