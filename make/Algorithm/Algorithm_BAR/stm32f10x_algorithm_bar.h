/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_bar.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.14
Description: declare the barometer function
Others:      none
Function List:
             1. static void Intertial_Filter_Predict(float dt, float x[3]);
             2. static void Intertial_Filter_Corrent(float e, float dt,
                                                     float x[3], int i, float w);
             3. void AltitudeCombineThread(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.11  modify the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_BAR_H__
#define __STM32F10X_ALGORITHM_BAR_H__

#include "stm32f10x.h"

/* us store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
#define ALT_THREAD_RPD 5000

typedef struct NAV_TT
{
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
}nav_t;

extern nav_t nav;
extern float z_est[3];  /* Estimate z vz az */
extern uint8_t landed;
extern uint8_t accUpdated;

static void Intertial_Filter_Predict(float dt, float x[3]);
static void Intertial_Filter_Corrent(float e, float dt, float x[3], int i, float w);
extern void AltitudeCombineThread(void);

#endif
