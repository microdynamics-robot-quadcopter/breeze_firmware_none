/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_system_battery.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: declare the battery operation function
Others:      none
Function List:
             1. int  Battery_GetAD(void);
             2. int  Battery_GetTemp(void);
             3. void Battery_Check(void);
             4. void Battery_CheckInit(void);
             5. u16  Battery_GetADC(u8 ch);
             6. u16  Battery_GetADCAverage(u8 ch, u8 times);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.30  modify the module
*******************************************************************************/

#ifndef __STM32F10X_SYSTEM_BATTERY_H__
#define __STM32F10X_SYSTEM_BATTERY_H__

#include "stm32f10x.h"

#define BAT_CHECK_PERIOD  5000  /* Unit: ms */
#define BAT_ALARM_VAL     3.65  /* Threshold of starting motor, starting motor will lead to 0.3-0.4v voltage drop */
#define BAT_CHARGE_VAL    1.0   /* Charge battery val unit: v */
#define BAT_OVERDIS_VAL   3.15  /* Over discharge protect value, below this value long time will lead to land */

typedef struct
{
    int   ADVal;
    int   OverDischargeCnt;
    char  AlarmFlag;
    char  ChargeState;
    float RealVal;
    float TestVal;     /* Practice value, measured by multimeter */
    float ADRefVal;    /* Supply voltage, about 3.3v, need to be measured */
    float ADInputVal;  /* The pad voltage between R15 and R17 */
    float Factor;      /* For voltage calibration */
}Bat_TypedefStructure;

extern Bat_TypedefStructure Battery;

extern int  Battery_GetAD(void);
extern int  Battery_GetTemp(void);
extern void Battery_Check(void);
extern void Battery_CheckInit(void);
extern u16  Battery_GetADC(u8 ch);
extern u16  Battery_GetADCAverage(u8 ch, u8 times);

#endif
