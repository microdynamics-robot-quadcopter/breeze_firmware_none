/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_flight.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.10.20
Description: Declare the flight function
Others:      none
Function List:
History:
<author>    <date>        <desc>
maksyuki    2017.01.11    Modify the module
myyerrol    2017.04.29    Format the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_FLIGHT__
#define __STM32F10X_ALGORITHM_FLIGHT__

#include <stdbool.h>
#include "stm32f10x.h"

// Unit: ms.
#define FLIGHT_LOST_RC_TIME_MAX  1000
#define FLIGHT_AUTOLAND_TIME_MAX 4000

extern bool flight_lost_rc_flag;

extern void Flight_SetMode(void);
extern void Flight_StartAutoland(void);
extern void Flight_HandleFailures(void);

#endif
