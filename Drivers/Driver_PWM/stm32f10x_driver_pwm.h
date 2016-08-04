/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_driver_pwm.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.4
Description: declare the PWM operation function
Others:      none
Function List:
             1. void PWM_Init(void); 
             2. void PWM_Flash(const u16 MOTO1_PWM, const u16 MOTO2_PWM, const u16 MOTO3_PWM, const u16 MOTO4_PWM);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_PWM_H__
#define __STM32F10X_DRIVER_PWM_H__

#include "stm32f10x.h"

void PWM_Init(void);
void PWM_Flash(const u16 MOTO1_PWM, const u16 MOTO2_PWM, const u16 MOTO3_PWM, const u16 MOTO4_PWM);

#endif
