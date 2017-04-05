/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_eeprom.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.21
Description: implement the eeprom function
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

#include "stm32f10x_driver_flash.h"
#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_control.h"

#define TABLE_ADDRESS (STM32_FLASH_BASE + STM32_FLASH_OFFEST + 0)

EEPROM_Table table;
uint8_t gParamsSaveEEPROMRequest = 0;

#define EEPROM_DEFAULT_VERSION 1

static uint8_t isEEPROMValid(void)
{
    STMFLASH_Read(TABLE_ADDRESS, (uint16_t *)(&table), 2);
    if ((int16_t)table.version == EEPROM_DEFAULT_VERSION)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void TableResetDefault(void)
{
    STMFLASH_Write(TABLE_ADDRESS, (uint16_t *)(&(table.version)), 2);
}

void TableReadEEPROM(void)
{
    uint8_t paramNums = sizeof(table) / sizeof(float);
    STMFLASH_Read(TABLE_ADDRESS, (uint16_t *)(&table), paramNums * 2);
}

void TableWriteEEPROM(void)
{
    uint8_t paramNums = sizeof(table) / sizeof(float);
    STMFLASH_Write(TABLE_ADDRESS, (uint16_t *)(&table), paramNums * 2);
}

extern u8 RX_ADDRESS[RX_ADR_WIDTH];
extern u8 NRFMatched;

void TableToParam(void)
{
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        ((float *)(&pitch_angle_PID))[i] = ((float *)(&table.pidPitch))[i];
        ((float *)(&roll_angle_PID))[i]  = ((float *)(&table.pidRoll))[i];
        ((float *)(&yaw_angle_PID))[i]   = ((float *)(&table.pidYaw))[i];

        ((float *)(&pitch_rate_PID))[i]  = ((float *)(&table.pidPitchRate))[i];
        ((float *)(&roll_rate_PID))[i]   = ((float *)(&table.pidRollRate))[i];
        ((float *)(&yaw_rate_PID))[i]    = ((float *)(&table.pidYawRate))[i];

        ((float *)(&alt_PID))[i]         = ((float *)(&table.pidAlt))[i];
        ((float *)(&alt_vel_PID))[i]     = ((float *)(&table.pidAltVel))[i];

        imu.accOffset[i]  = table.accOffset[i];
        imu.gyroOffset[i] = table.gyroOffset[i];
    }

    for (i = 0; i < 5; i++)
    {
        ((u8 *)(&RX_ADDRESS))[i] = ((float *)(&table.NRFAddr))[i];
    }

    //BTstate = table.BTState;
    NRFMatched = table.NRFMatchFlag;
}

void ParamToTable(void)
{
    uint8_t i;
    //float temp;
    for (i = 0; i < 3; i++)
    {
        ((float *)(&table.pidPitch))[i] = ((float *)(&pitch_angle_PID))[i];
        //temp=((float *)(&roll_angle_PID))[i];
        *((float *)(&table.pidRoll) + i) = ((float *)(&roll_angle_PID))[i]; /* the same with next line Bug? */
        ((float *)(&table.pidRoll))[i]   = ((float *)(&roll_angle_PID))[i];
        ((float *)(&table.pidYaw))[i]    = ((float *)(&yaw_angle_PID))[i];

        ((float *)(&table.pidPitchRate))[i] = ((float *)(&pitch_rate_PID))[i];
        ((float *)(&table.pidRollRate))[i]  = ((float *)(&roll_rate_PID))[i];
        ((float *)(&table.pidYawRate))[i]   = ((float *)(&yaw_rate_PID))[i];

        ((float *)(&table.pidAlt))[i]       = ((float *)(&alt_PID))[i];
        ((float *)(&table.pidAltVel))[i]    = ((float *)(&alt_vel_PID))[i];

        table.accOffset[i]  = imu.accOffset[i];
        table.gyroOffset[i] = imu.gyroOffset[i];
    }

    for (i = 0; i < 5; i++)
    {
        ((float *)(&table.NRFAddr))[i] = ((u8 *)(&RX_ADDRESS))[i];
    }

    //table.BTState = BTstate;
    table.NRFMatchFlag = NRFMatched;
}

void LoadParamsFromEEPROM(void)
{
    if (isEEPROMValid())
    {
        TableReadEEPROM();
        TableToParam();
    }
    else
    {
        ParamSetDefault();
        ParamToTable();
        table.version = EEPROM_DEFAULT_VERSION;
        TableWriteEEPROM();
    }
}

void SaveParamsToEEPROM(void)
{
    ParamToTable();
    TableWriteEEPROM();
}

void ParamSetDefault(void)
{
    pitch_angle_PID.P = 3.5;
    pitch_angle_PID.I = 0;          /* 1.0 or 0 */
    pitch_angle_PID.D = 0;
    pitch_angle_PID.iLimit = 300;   /* or 1000 */

    pitch_rate_PID.P  = 0.7;
    pitch_rate_PID.I  = 0.5;
    pitch_rate_PID.D  = 0.03;
    pitch_rate_PID.iLimit = 300;

    roll_angle_PID.P = 3.5;
    roll_angle_PID.I = 0;           /* 1.0 */
    roll_angle_PID.D = 0;
    roll_angle_PID.iLimit = 300;    /* or 1000 */

    roll_rate_PID.P  = 0.7;
    roll_rate_PID.I  = 0.5;
    roll_rate_PID.D  = 0.03;
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

    /* should chango to read eeprom cfg. should be 0 */
    imu.accOffset[0] = -0.1620515;
    imu.accOffset[1] = 0.07422026;
    imu.accOffset[2] = 0.7743073;

    imu.gyroOffset[0] = -0.06097556;
    imu.gyroOffset[1] = -0.03780485;
    imu.gyroOffset[2] = 0;
}
