/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_motor.h
Author:      myyerrol
Version:     none
Create date: 2017.04.23
Description: Declare the motor operation function
Others:      none
Function List:
             1. void Motor_Init(void);
             2. void Motor_SetPWM(s16 motor1_pwm, s16 motor2_pwm,
                                  s16 motor3_pwm, s16 motor4_pwm)
History:
<author>    <date>        <desc>
myyerrol    2017.04.23    Modify the module
*******************************************************************************/


#ifndef __STM32F10X_MODULE_MOTOR_H__
#define __STM32F10X_MODULE_MOTOR_H__

#include "stm32f10x.h"

#define MOTOR_PWM_MAXVALUE 999

extern void Motor_Init(void);
extern void Motor_SetPWM(s16 motor1_pwm, s16 motor2_pwm, s16 motor3_pwm,
                         s16 motor4_pwm);

#endif
