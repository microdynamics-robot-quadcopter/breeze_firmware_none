/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_pwm.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.8.4
Description: declare the PWM operation function
Others:      none
Function List:
             1. void PWM_Init(void);
             2. void PWM_MotorFlash(int16_t MOTO1_PWM, int16_t MOTO2_PWM,
                                    int16_t MOTO3_PWM, int16_t MOTO4_PWM);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.04  modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_PWM_H__
#define __STM32F10X_DRIVER_PWM_H__

#include "stm32f10x.h"

#define PWM_MAXVALUE 999

extern void PWM_Init(void);
extern void PWM_MotorFlash(int16_t MOTO1_PWM, int16_t MOTO2_PWM, int16_t MOTO3_PWM, int16_t MOTO4_PWM);

#endif
