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
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_algorithm_imu.h"

// Request to save the parameters to the EEPROM.
bool eeprom_params_request_flag = false;
EEPROM_Table EEPROM_TableStructure;

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
    Control_PIDPitchAngle.kp        = 3.5;
    Control_PIDPitchAngle.ki        = 0;
    Control_PIDPitchAngle.kd        = 0;
    Control_PIDPitchAngle.limit_int = 300;

    Control_PIDPitchAngleRate.kp        = 0.7;
    Control_PIDPitchAngleRate.ki        = 0.5;
    Control_PIDPitchAngleRate.kd        = 0.03;
    Control_PIDPitchAngleRate.limit_int = 300;

    Control_PIDRollAngle.kp        = 3.5;
    Control_PIDRollAngle.ki        = 0;
    Control_PIDRollAngle.kd        = 0;
    Control_PIDRollAngle.limit_int = 300;

    Control_PIDRollAngleRate.kp        = 0.7;
    Control_PIDRollAngleRate.ki        = 0.5;
    Control_PIDRollAngleRate.kd        = 0.03;
    Control_PIDRollAngleRate.limit_int = 300;

    Control_PIDYawAngle.kp = 1;
    Control_PIDYawAngle.ki = 0.2;
    Control_PIDYawAngle.kd = 0;

    Control_PIDYawAngleRate.kp = 20;
    Control_PIDYawAngleRate.ki = 0;
    Control_PIDYawAngleRate.kd = 0;

    Control_PIDAlt.kp = 1.0;
    Control_PIDAlt.ki = 0;
    Control_PIDAlt.kd = 0;

    Control_PIDAltVel.kp = 0.1f;
    Control_PIDAltVel.ki = 0.02f;
    Control_PIDAltVel.kd = 0;

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
            ((float *)(&Control_PIDRollAngle))[i];
        ((float *)(&EEPROM_TableStructure.pid_pitch))[i]      =
            ((float *)(&Control_PIDPitchAngle))[i];
        ((float *)(&EEPROM_TableStructure.pid_yaw))[i]        =
            ((float *)(&Control_PIDYawAngle))[i];

        ((float *)(&EEPROM_TableStructure.pid_roll_rate))[i]  =
            ((float *)(&Control_PIDRollAngleRate))[i];
        ((float *)(&EEPROM_TableStructure.pid_pitch_rate))[i] =
            ((float *)(&Control_PIDPitchAngleRate))[i];
        ((float *)(&EEPROM_TableStructure.pid_yaw_rate))[i]   =
            ((float *)(&Control_PIDYawAngleRate))[i];

        ((float *)(&EEPROM_TableStructure.pid_alt))[i]        =
            ((float *)(&Control_PIDAlt))[i];
        ((float *)(&EEPROM_TableStructure.pid_alt_vel))[i]    =
            ((float *)(&Control_PIDAltVel))[i];

        EEPROM_TableStructure.offset_acc[i] = imu.accOffset[i];
        EEPROM_TableStructure.offset_gyr[i] = imu.gyroOffset[i];
    }

    for (i = 0; i < 5; i++)
    {
        ((float *)(&EEPROM_TableStructure.nrf_addr))[i] =
            ((u8 *)(&nrf24l01_rx_address))[i];
    }

    EEPROM_TableStructure.nrf_matched_flag = nrf24l01_matched_flag;
}

void EEPROM_TransTableToParams(void)
{
    u8 i;

    for (i = 0; i < 3; i++)
    {
        ((float *)(&Control_PIDRollAngle))[i]  =
            ((float *)(&EEPROM_TableStructure.pid_roll))[i];
        ((float *)(&Control_PIDPitchAngle))[i] =
            ((float *)(&EEPROM_TableStructure.pid_pitch))[i];
        ((float *)(&Control_PIDYawAngle))[i]   =
            ((float *)(&EEPROM_TableStructure.pid_yaw))[i];

        ((float *)(&Control_PIDRollAngleRate))[i]   =
            ((float *)(&EEPROM_TableStructure.pid_roll_rate))[i];
        ((float *)(&Control_PIDPitchAngleRate))[i]  =
            ((float *)(&EEPROM_TableStructure.pid_pitch_rate))[i];
        ((float *)(&Control_PIDYawAngleRate))[i]    =
            ((float *)(&EEPROM_TableStructure.pid_yaw_rate))[i];

        ((float *)(&Control_PIDAlt))[i]         =
            ((float *)(&EEPROM_TableStructure.pid_alt))[i];
        ((float *)(&Control_PIDAltVel))[i]     =
            ((float *)(&EEPROM_TableStructure.pid_alt_vel))[i];

        imu.accOffset[i]  = EEPROM_TableStructure.offset_acc[i];
        imu.gyroOffset[i] = EEPROM_TableStructure.offset_gyr[i];
    }

    for (i = 0; i < 5; i++)
    {
        ((u8 *)(&nrf24l01_rx_address))[i] =
            ((float *)(&EEPROM_TableStructure.nrf_addr))[i];
    }

    nrf24l01_matched_flag = EEPROM_TableStructure.nrf_matched_flag;
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
