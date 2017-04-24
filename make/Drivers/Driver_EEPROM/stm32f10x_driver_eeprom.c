/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_eeprom.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.21
Description: Implement the eeprom function
Others:      none
Function List:
             1. void EEPROM_LoadParamsFromEEPROM(void);
             2. void EEPROM_ReadTableFromEEPROM(void);
             3. void EEPROM_SaveParamsToEEPROM(void);
             4. void EEPROM_SetDefaultParams(void);
             5. void EEPROM_TransParamsToTable(void);
             6. void EEPROM_TransTableToParams(void);
             7. void EEPROM_WriteTableToEEPROM(void);
             8. u8   EEPROM_IsValid(void);
History:
<author>    <date>        <desc>
maksyuki    2016.12.19    Modify the module
myyerrol    2017.04.22    Format the module
*******************************************************************************/

#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_driver_flash.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_control.h"

EEPROM_Table EEPROM_TableStructure;
// Request to save the parameters to the EEPROM.
u8 eeprom_params_request = 0;

void EEPROM_LoadParamsFromEEPROM(void)
{
    if (EEPROM_IsValid())
    {
        EEPROM_ReadTableFromEEPROM();
        EEPROM_TransTableToParams();
    }
    else
    {
        EEPROM_SetDefaultParams();
        EEPROM_TransParamsToTable();
        EEPROM_TableStructure.version = EEPROM_DEFAULT_VERSION;
        EEPROM_WriteTableToEEPROM();
    }
}

extern void EEPROM_ResetDefaultTable(void)
{
    Flash_Write(EEPROM_TABLE_ADDRESS,
               (u16 *)(&(EEPROM_TableStructure.version)), 2);
}

void EEPROM_ReadTableFromEEPROM(void)
{
    u8 params_num = sizeof(EEPROM_TableStructure) / sizeof(float);
    Flash_Read(EEPROM_TABLE_ADDRESS, (u16 *)(&EEPROM_TableStructure),
               params_num * 2);
}

void EEPROM_SaveParamsToEEPROM(void)
{
    EEPROM_TransParamsToTable();
    EEPROM_WriteTableToEEPROM();
}

void EEPROM_SetDefaultParams(void)
{
    pitch_angle_PID.P      = 3.5;
    pitch_angle_PID.I      = 0;
    pitch_angle_PID.D      = 0;
    pitch_angle_PID.iLimit = 300;

    pitch_rate_PID.P      = 0.7;
    pitch_rate_PID.I      = 0.5;
    pitch_rate_PID.D      = 0.03;
    pitch_rate_PID.iLimit = 300;

    roll_angle_PID.P      = 3.5;
    roll_angle_PID.I      = 0;
    roll_angle_PID.D      = 0;
    roll_angle_PID.iLimit = 300;

    roll_rate_PID.P      = 0.7;
    roll_rate_PID.I      = 0.5;
    roll_rate_PID.D      = 0.03;
    roll_rate_PID.iLimit = 300;

    yaw_angle_PID.P = 1;
    yaw_angle_PID.I = 0.2;
    yaw_angle_PID.D = 0;

    yaw_rate_PID.P  = 20;
    yaw_rate_PID.I  = 0;
    yaw_rate_PID.D  = 0;

    alt_PID.P = 1.0;
    alt_PID.I = 0;
    alt_PID.D = 0;

    alt_vel_PID.P = 0.1f;
    alt_vel_PID.I = 0.02f;
    alt_vel_PID.D = 0;

    imu.accOffset[0] = -0.1620515;
    imu.accOffset[1] = 0.07422026;
    imu.accOffset[2] = 0.7743073;

    imu.gyroOffset[0] = -0.06097556;
    imu.gyroOffset[1] = -0.03780485;
    imu.gyroOffset[2] = 0;
}

void EEPROM_TransParamsToTable(void)
{
    u8 i;

    for (i = 0; i < 3; i++)
    {
        ((float *)(&EEPROM_TableStructure.pid_roll))[i]       =
            ((float *)(&roll_angle_PID))[i];
        ((float *)(&EEPROM_TableStructure.pid_pitch))[i]      =
            ((float *)(&pitch_angle_PID))[i];
        ((float *)(&EEPROM_TableStructure.pid_yaw))[i]        =
            ((float *)(&yaw_angle_PID))[i];

        ((float *)(&EEPROM_TableStructure.pid_roll_rate))[i]  =
            ((float *)(&roll_rate_PID))[i];
        ((float *)(&EEPROM_TableStructure.pid_pitch_rate))[i] =
            ((float *)(&pitch_rate_PID))[i];
        ((float *)(&EEPROM_TableStructure.pid_yaw_rate))[i]   =
            ((float *)(&yaw_rate_PID))[i];

        ((float *)(&EEPROM_TableStructure.pid_alt))[i]        =
            ((float *)(&alt_PID))[i];
        ((float *)(&EEPROM_TableStructure.pid_alt_vel))[i]    =
            ((float *)(&alt_vel_PID))[i];

        EEPROM_TableStructure.offset_acc[i] = imu.accOffset[i];
        EEPROM_TableStructure.offset_gyr[i] = imu.gyroOffset[i];
    }

    for (i = 0; i < 5; i++)
    {
        ((float *)(&EEPROM_TableStructure.nrf_addr))[i] =
            ((u8 *)(&RX_ADDRESS))[i];
    }

    EEPROM_TableStructure.nrf_match_flag = NRFMatched;
}

void EEPROM_TransTableToParams(void)
{
    u8 i;

    for (i = 0; i < 3; i++)
    {
        ((float *)(&roll_angle_PID))[i]  =
            ((float *)(&EEPROM_TableStructure.pid_roll))[i];
        ((float *)(&pitch_angle_PID))[i] =
            ((float *)(&EEPROM_TableStructure.pid_pitch))[i];
        ((float *)(&yaw_angle_PID))[i]   =
            ((float *)(&EEPROM_TableStructure.pid_yaw))[i];

        ((float *)(&roll_rate_PID))[i]   =
            ((float *)(&EEPROM_TableStructure.pid_roll_rate))[i];
        ((float *)(&pitch_rate_PID))[i]  =
            ((float *)(&EEPROM_TableStructure.pid_pitch_rate))[i];
        ((float *)(&yaw_rate_PID))[i]    =
            ((float *)(&EEPROM_TableStructure.pid_yaw_rate))[i];

        ((float *)(&alt_PID))[i]         =
            ((float *)(&EEPROM_TableStructure.pid_alt))[i];
        ((float *)(&alt_vel_PID))[i]     =
            ((float *)(&EEPROM_TableStructure.pid_alt_vel))[i];

        imu.accOffset[i]  = EEPROM_TableStructure.offset_acc[i];
        imu.gyroOffset[i] = EEPROM_TableStructure.offset_gyr[i];
    }

    for (i = 0; i < 5; i++)
    {
        ((u8 *)(&RX_ADDRESS))[i] =
            ((float *)(&EEPROM_TableStructure.nrf_addr))[i];
    }

    NRFMatched = EEPROM_TableStructure.nrf_match_flag;
}

void EEPROM_WriteTableToEEPROM(void)
{
    u8 params_num = sizeof(EEPROM_TableStructure) / sizeof(float);
    Flash_Write(EEPROM_TABLE_ADDRESS, (u16 *)(&EEPROM_TableStructure),
                params_num * 2);
}

extern u8 EEPROM_IsValid(void)
{
    Flash_Read(EEPROM_TABLE_ADDRESS, (u16 *)(&EEPROM_TableStructure), 2);

    if ((s16)EEPROM_TableStructure.version == EEPROM_DEFAULT_VERSION)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
