/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_flight.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.10.20
Description: implement the flight function
Others:      none
Function List:
             1. void FlightStateSet(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.11  modify the module
*******************************************************************************/

#include "stm32f10x_module_rpdata.h"
#include "stm32f10x_algorithm_bar.h"
#include "stm32f10x_algorithm_flight.h"
#include "stm32f10x_algorithm_control.h"

/* For copter launch and flight mode switch. it's raw and simple for climb rate mode now */
/* TOBE IMPROVED */
void FlightStateSet(void)
{
    if (FLY_ENABLE)
    {
        if (NRF_Data.throttle >= 600)
        {
            if (altCtrlMode != CLIMB_RATE)
            {
                zIntReset   = 1;
                thrustZSp   = 0;
                altCtrlMode = CLIMB_RATE;
                offLandFlag = 1;
                altLand     = -nav.z;    /* Record the height of take-off */
                SetHeadFree(1);
            }
        }
        else
        {
            if (altCtrlMode == MANUAL)
            {
                NRF_Data.throttle = 200; /* Idling 200 */
            }
        }
    }
}
