/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics.

Filename:    stm32f10x_driver_pwm.h
Author:      maksyuki
Version:     1.0
Date:        2016.8.4
Description: declare the PWM operation function
Others:      none
Function List:
             1. extern void PWM_Init(void); 
             2. extern void PWM_Flash(u16 MOTO1_PWM, u16 MOTO2_PWM, u16 MOTO3_PWM, u16 MOTO4_PWM);
History:     none
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_PWM_H__
#define __STM32F10X_DRIVER_PWM_H__

#include "stm32f10x.h"

#define PWM_MAXVALUE 999

extern void PWM_Init(void);
extern void PWM_MotorFlash(u16 MOTO1_PWM, u16 MOTO2_PWM, u16 MOTO3_PWM, u16 MOTO4_PWM);

#endif
