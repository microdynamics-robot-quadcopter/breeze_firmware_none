/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_eeprom.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.21
Description: Declare the eeprom function
Others:      none
Function List:
             1. void EEPROM_LoadParamsFromEEPROM(void);
             2. void EEPROM_ResetDefaultTable(void)
             3. void EEPROM_ReadTableFromEEPROM(void);
             4. void EEPROM_SaveParamsToEEPROM(void);
             5. void EEPROM_SetDefaultParams(void);
             6. void EEPROM_TransParamsToTable(void);
             7. void EEPROM_TransTableToParams(void);
             8. void EEPROM_WriteTableToEEPROM(void);
             9. u8   EEPROM_IsValid(void);
History:
<author>    <date>        <desc>
maksyuki    2016.12.19    Modify the module
myyerrol    2017.04.22    Format the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_EEPROM_H__
#define __STM32F10X_DRIVER_EEPROM_H__

#include "stm32f10x.h"

#define EEPROM_DEFAULT_VERSION 1
#define EEPROM_TABLE_ADDRESS   (FLASH_BASE + FLASH_OFFEST + 0)

typedef struct
{
    float nrf_addr[5];
    float nrf_match_flag;
    float offset_acc[3];
    float offset_gyr[3];
    float offset_mag[3];
    float pid_alt[3];
    float pid_alt_vel[3];
    float pid_roll[3];
    float pid_roll_rate[3];
    float pid_pitch[3];
    float pid_pitch_rate[3];
    float pid_yaw[3];
    float pid_yaw_rate[3];
    float version;
} EEPROM_Table;

extern EEPROM_Table EEPROM_TableStructure;;
extern u8 eeprom_params_request;

extern void EEPROM_LoadParamsFromEEPROM(void);
extern void EEPROM_ResetDefaultTable(void);
extern void EEPROM_ReadTableFromEEPROM(void);
extern void EEPROM_SaveParamsToEEPROM(void);
extern void EEPROM_SetDefaultParams(void);
extern void EEPROM_TransParamsToTable(void);
extern void EEPROM_TransTableToParams(void);
extern void EEPROM_WriteTableToEEPROM(void);
extern u8   EEPROM_IsValid(void);

#endif
