/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_battery.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Declare the battery operation function
Others:      none
Function List:
             1. void Battery_Check(void);
             2. void Battery_Init(void);
             3. u16  Battery_GetADC(u8 ch);
             4. u16  Battery_GetADCAverage(u8 ch, u8 times);
             5. int  Battery_GetAD(void);
             6. int  Battery_GetTemp(void);

History:
<author>    <date>        <desc>
maksyuki    2016.12.30    Modify the module
myyerrol    2017.04.11    Format the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_BATTERY_H__
#define __STM32F10X_MODULE_BATTERY_H__

#include "stm32f10x.h"

// The Period of battery check. Unit is ms.
#define BATTERY_CHECK_PERIOD    5000
// The threshold of starting motors.
// Starting motors will lead voltage to 0.3-0.4v drop.
#define BATTERY_VOLTAGE_ALARM   3.65
// The charged voltage.
#define BATTERY_VOLTAGE_CHARGE  1.0
// The protective voltage of over discharge.
// Below the value long time will to lead quadcopter to land.
#define BATTERY_VOLTAGE_OVERDIS 3.15

typedef enum
{
    TRUE  = 0,
    FALSE = 1
} bool;

typedef struct
{
    bool  flag_alarm;
    bool  flag_charge;
    int   over_discharge_cnt;
    int   voltage_ad;
    float voltage_ad_ref;
    float voltage_ad_in;
    float voltage_calculate;
    float voltage_factor;
    float voltage_measure;
} Battery_Information;

extern Battery_Information Battery_InformationStructure;

extern void Battery_Check(void);
extern void Battery_Init(void);
extern u16  Battery_GetADC(u8 ch);
extern u16  Battery_GetADCAverage(u8 ch, u8 times);
extern int  Battery_GetAD(void);
extern int  Battery_GetTemp(void);

#endif
