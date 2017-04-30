/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_altitude.h
Author:      myyerrol
Version:     none
Create date: 2017.04.30
Description: Declare the altitude function
Others:      none
Function List:
             1. void Altitude_CombineData(void);
             2. void Altitude_CorrectByUsingInertialFilter(float delta_time,
                                                           float data[3],
                                                           float weight,
                                                           float corr_factor,
                                                           u8 i);
             3. void Altitude_PredictByUsingInertialFilter(float delta_time,
                                                           float data[3]);
History:
<author>    <date>        <desc>
myyerrol    2017.04.30    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_ALTITUDE_H__
#define __STM32F10X_ALGORITHM_ALTITUDE_H__

#include <stdbool.h>
#include "stm32f10x.h"

typedef struct
{
    float pos_x;
    float pos_y;
    float pos_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float acc_x;
    float acc_y;
    float acc_z;
} Altitude_NEDFrame;

extern bool altitude_acc_update_flag;
extern Altitude_NEDFrame Altitude_NEDFrameStructure;

extern void Altitude_CombineData(void);
extern void Altitude_CorrectByUsingInertialFilter(float delta_time,
                                                  float data[3],
                                                  float weight,
                                                  float corr_factor,
                                                  u8 i);
extern void Altitude_PredictByUsingInertialFilter(float delta_time,
                                                  float data[3]);

#endif
