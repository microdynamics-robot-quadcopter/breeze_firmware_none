/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_eeprom.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.21
Description: declare the eeprom function
Others:      none
Function List:
             1. void LoadParamsFromEEPROM(void);
             2. void ParamSetDefault(void);
             3. void ParamToTable(void);
             4. void TableToParam(void);
             5. void TableWriteEEPROM(void);
             6. void TableReadEEPROM(void);
             7. void SaveParamsToEEPROM(void);
             8. uint8_t isEEPROMValid(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.19  modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_EEPROM_H__
#define __STM32F10X_DRIVER_EEPROM_H__

#include "stm32f10x.h"

typedef struct EEPROM
{
    float pidPitch[3];
    float pidPitchRate[3];
    float pidYaw[3];
    float pidYawRate[3];
    float pidRoll[3];
    float pidRollRate[3];
    float pidAlt[3];
    float pidAltVel[3];
    float accOffset[3];
    float gyroOffset[3];
    float magOffset[3];
    float NRFAddr[5];
    float version;
    float BTState;
    float NRFMatchFlag;
}EEPROM_Table;

extern EEPROM_Table table;
extern uint8_t gParamsSaveEEPROMRequest;

extern void LoadParamsFromEEPROM(void);
extern void ParamSetDefault(void);
extern void ParamToTable(void);
extern void TableToParam(void);
extern void TableWriteEEPROM(void);
extern void TableReadEEPROM(void);
extern void SaveParamsToEEPROM(void);

static uint8_t isEEPROMValid(void);

#endif
