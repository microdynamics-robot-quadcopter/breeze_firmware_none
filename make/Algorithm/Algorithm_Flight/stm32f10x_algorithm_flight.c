/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_flight.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.10.20
Description: Implement the flight function
Others:      none
Function List:
             1. void Flight_SetMode(void);
             2. void Flight_StartAutoland(void);
             3. void Flight_HandleFailures(void);
History:
<author>    <date>        <desc>
maksyuki    2017.01.11    Modify the module
myyerrol    2017.04.29    Format the module
*******************************************************************************/

#include <math.h>
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_motor.h"
#include "stm32f10x_algorithm_altitude.h"
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_algorithm_flight.h"
#include "stm32f10x_algorithm_imu.h"

bool flight_lost_rc_flag;

void Flight_SetMode(void)
{
    if (comm_link_fly_enable_flag)
    {
        if (CommLink_DataStructure.thr >= 600)
        {
            if (altCtrlMode != CLIMB_RATE)
            {
                zIntReset   = 1;
                thrustZSp   = 0;
                altCtrlMode = CLIMB_RATE;
                offLandFlag = 1;
                altLand     = -Altitude_NEDFrameStructure.pos_z;
                SetHeadFree(1);
            }
        }
        else
        {
            if (altCtrlMode == MANUAL)
            {
                CommLink_DataStructure.thr = 200;
            }
        }
    }
}

void Flight_StartAutoland(void)
{
    static u32 land_start_timestamp = 0;
    u32 land_time = 0;

    if (offLandFlag)
    {
        if (land_start_timestamp == 0)
        {
            land_start_timestamp = Delay_GetRuntimeMs();
        }
        land_time = Delay_GetRuntimeMs() - land_start_timestamp;
        if (land_time > FLIGHT_AUTOLAND_TIME_MAX)
        {
            altCtrlMode               = MANUAL;
            comm_link_fly_enable_flag = false;
            offLandFlag               = 0;
            land_start_timestamp      = 0;
        }
    }
    else
    {
        altCtrlMode               = MANUAL;
        comm_link_fly_enable_flag = false;
    }
}

void Flight_HandleFailures(void)
{
    static u32 new_timestamp = 0;
    u16 lost_rc_time  = 0;

    // When quadcopter crash down or excessive pitch or roll angles are
    // detected, turn off the motor, to prevent the motor burning.
    if (fabs(imu.pitch) > 80 || fabs(imu.roll) > 80)
    {
        Motor_SetPWM(0, 0, 0, 0);
        comm_link_fly_enable_flag = false;
    }

    // Lose RC signal.
    new_timestamp = Delay_GetRuntimeMs();

    if (new_timestamp > comm_link_last_rc_timestamp)
    {
        lost_rc_time = new_timestamp - comm_link_last_rc_timestamp;
    }
    else
    {
        lost_rc_time = 65536 - comm_link_last_rc_timestamp + new_timestamp;
    }
    if (lost_rc_time > FLIGHT_LOST_RC_TIME_MAX)
    {
        if (offLandFlag || comm_link_fly_enable_flag)
        {
            altCtrlMode = LANDING;
        }
        flight_lost_rc_flag = true;
    }
    else
    {
        flight_lost_rc_flag = false;
    }
}
