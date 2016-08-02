#ifndef STM32F10X_DRIVER_LED_H
#define STM32F10X_DRIVER_LED_H

#include "sys.h"

#define LED0 PBout(5)

void LED_Init(void);

#endif // STM32F10X_DRIVER_LED_H
