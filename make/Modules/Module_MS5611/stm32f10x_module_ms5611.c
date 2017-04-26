/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_ms5611.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.01
Description: Implement the ms5611 function
Others:      none
Function List:
             1.  void  MS5611_AddNewAltitude(float value);
             2.  void  MS5611_AddNewTemperature(float value);
             3.  void  MS5611_AddNewPressure(float value);
             4.  void  MS5611_GetPressure(void);
             5.  void  MS5611_GetTemperature(void);
             6.  void  MS5611_Init(void);
             7.  void  MS5611_ReadPROM(void);
             8.  void  MS5611_Reset(void);
             9.  void  MS5611_StartConversion(u8 command);
             10. void  MS5611_UpdateData(void);
             11. u8    MS5611_WaitBaroInitOffset(void);
             12. u32   MS5611_GetConversion(void);
             13. float MS5611_GetAltitude(void);
             14. float MS5611_GetAverage(float *buffer, int size);
History:
<author>    <date>        <desc>
maksyuki    2017.01.01    Modify the module
myyerrol    2017.04.24    Format the module
*******************************************************************************/

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_it.h"

static u8  current_state       = MS5611_STATE_START_CONVERT_TEMP;
static u16 calibration_params[MS5611_PROM_REG_COUNT];
static u32 start_convert_timestamp;
static u32 conversion_delay_us = 0;
static s32 temperature_value;

static u16    pressure_init_cnt   = 0;
// Save the altitude of 0m(relative).
static float  altitude_offset     = 0;
// Save the pressure of 0m(relative).
static float  pressure_offset     = 0;
static double pressure_offset_num = 0;

bool ms5611_pressure_offset_flag  = false;
bool ms5611_altitude_update_flag  = false;

volatile float ms5611_altitude    = 0;
volatile float ms5611_pressure    = 0;
volatile float ms5611_temperature = 0;

// Delay table: different sampling precision is with different delay time.
u32 delay_us_table[9] = {
    1500,  // MS5611_OSR_256  0.9ms  0x00
    1500,  // MS5611_OSR_256  0.9ms
    2000,  // MS5611_OSR_512  1.2ms  0x02
    2000,  // MS5611_OSR_512  1.2ms
    3000,  // MS5611_OSR_1024 2.3ms  0x04
    3000,  // MS5611_OSR_1024 2.3ms
    5000,  // MS5611_OSR_2048 4.6ms  0x06
    5000,  // MS5611_OSR_2048 4.6ms
    11000, // MS5611_OSR_4096 9.1ms  0x08
};

// FIFO queue.
static float temperature_buffer[MS5611_BUFFER_SIZE];
static float pressure_buffer[MS5611_BUFFER_SIZE];
static float altitude_buffer[MS5611_BUFFER_SIZE];

static u8 index_temperature = 0;
static u8 index_pressure    = 0;

void MS5611_AddNewAltitude(float value)
{
    s16 i;

    for (i = 1; i < MS5611_BUFFER_SIZE; i++)
    {
        altitude_buffer[i-1] = altitude_buffer[i];
    }

    altitude_buffer[MS5611_BUFFER_SIZE - 1] = value;
}

void MS5611_AddNewTemperature(float value)
{
    temperature_buffer[index_temperature] = value;
    index_temperature = (index_temperature + 1) % MS5611_BUFFER_SIZE;
}

void MS5611_AddNewPressure(float value)
{
    pressure_buffer[index_pressure] = value;
    index_pressure = (index_pressure + 1) % MS5611_BUFFER_SIZE;
}

// Read the conversion result of barometer and compensate the results.
void MS5611_GetPressure(void)
{
    s32 pressure_raw;
    s64 actual_offset;
    s64 actual_sensitivity;
    s64 actual_temperature;
    s64 delta;
    s64 temp;
    s64 temp_offset;
    s64 temp_sensitivity;
    s64 temp_temperature;

    pressure_raw = MS5611_GetConversion();
    delta        = temperature_value - (((s32)calibration_params[4]) << 8);

    actual_offset      = (((s64)calibration_params[1]) << 16)
        + ((((s64)calibration_params[3]) * delta) >> 7);
    actual_sensitivity = (((s64)calibration_params[0]) << 15)
        + (((s64)(calibration_params[2]) * delta) >> 8);
    actual_temperature = 2000 + (delta * (s64)calibration_params[5]) / 8388608;

    // Compensate temperature secondly.
    if (actual_temperature < 2000)
	{
        temp               = (actual_temperature - 2000)
            * (actual_temperature - 2000);
        temp_offset        = (5 * temp) >> 1;
        temp_sensitivity   = (5 * temp) >> 2;
        temp_temperature   = (((s64)delta) * delta) >> 31;
        actual_temperature = actual_temperature - temp_temperature;
        actual_offset      = actual_offset - temp_offset;
        actual_sensitivity = actual_sensitivity - temp_sensitivity;
    }

    MS5611_AddNewTemperature(actual_temperature * 0.01F);

    ms5611_altitude = MS5611_GetAltitude();
    ms5611_pressure = (((((s64)pressure_raw) * actual_sensitivity) >> 21)
        - actual_offset) / 32768;
    ms5611_temperature = MS5611_GetAverage(temperature_buffer,
                                           MS5611_BUFFER_SIZE);
}

void MS5611_GetTemperature(void)
{
    temperature_value = MS5611_GetConversion();
}

void MS5611_Init(void)
{
    MS5611_Reset();
    Delay_TimeMs(100);
    MS5611_ReadPROM();
}

// Read the calibration values of MS5611 for correcting temperature and
// pressure.
void MS5611_ReadPROM(void)
{
    u8 value_h, value_l;
    u8 i;

    for (i = 0; i < MS5611_PROM_REG_COUNT; i++)
    {
        IIC_SendStartSignal();
        IIC_WriteOneByte(MS5611_ADDR);
        IIC_WaitAckSignal();
        IIC_WriteOneByte(MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));
        IIC_WaitAckSignal();
        IIC_SendStopSignal();
        Delay_TimeUs(5);
        IIC_SendStartSignal();
        // Enter receiving mode.
        IIC_WriteOneByte(MS5611_ADDR + 1);
        Delay_TimeUs(1);
        IIC_WaitAckSignal();
        // Read data with ACK.
        value_h = IIC_ReadOneByte(1);
        Delay_TimeUs(1);
        // Read date with NACK.
        value_l = IIC_ReadOneByte(0);
        IIC_SendStopSignal();
        calibration_params[i] = (((u16)value_h << 8) | value_l);
    }
}

// Send reset instruction to MS5611.
void MS5611_Reset(void)
{
    IIC_SendStartSignal();
    IIC_WriteOneByte(MS5611_ADDR);
    IIC_WaitAckSignal();
    IIC_WriteOneByte(MS5611_RESET);
    IIC_WaitAckSignal();
    IIC_SendStopSignal();
}

// Send starting conversion instruction to MS5611.
// Argument command:
// [MS5611_D1: conversion pressure]
// [MS5611_D2: conversion temperature]
void MS5611_StartConversion(u8 command)
{
    IIC_SendStartSignal();
    IIC_WriteOneByte(MS5611_ADDR);
    IIC_WaitAckSignal();
    IIC_WriteOneByte(command);
    IIC_WaitAckSignal();
    IIC_SendStopSignal();
}

// DFA for updating pressure and temperature.
void MS5611_UpdateData(void)
{
    switch (current_state)
    {
        case MS5611_STATE_START_CONVERT_TEMP:
        {
            MS5611_StartConversion(MS5611_D2 + MS5611_OSR_TEMP);
            conversion_delay_us = delay_us_table[MS5611_OSR_TEMP];
            start_convert_timestamp = Delay_GetRuntimeUs();
            current_state = MS5611_STATE_CONVERTING_TEMP;
            break;
        }
        case MS5611_STATE_CONVERTING_TEMP:
        {
            if ((Delay_GetRuntimeUs() - start_convert_timestamp) >
                conversion_delay_us)
            {
                MS5611_GetTemperature();
                current_state = MS5611_STATE_START_CONVERT_PRES;
            }
            break;
        }
        case MS5611_STATE_START_CONVERT_PRES:
        {
            MS5611_StartConversion(MS5611_D1 + MS5611_OSR_PRES);
            conversion_delay_us = delay_us_table[MS5611_OSR_PRES];
            start_convert_timestamp = Delay_GetRuntimeUs();
            current_state = MS5611_STATE_CONVERTING_PRES;
            break;
        }
        case MS5611_STATE_CONVERTING_PRES:
        {
            if ((Delay_GetRuntimeUs() - start_convert_timestamp) >
                conversion_delay_us)
            {
                MS5611_GetPressure();
                ms5611_altitude_update_flag = true;
                current_state = MS5611_STATE_START_CONVERT_TEMP;
            }
            break;
        }
        default:
        {
            current_state = MS5611_STATE_START_CONVERT_TEMP;
            break;
        }
    }
}

u8 MS5611_WaitBaroInitOffset(void)
{
    u32 timestamp_now   = 0;
    u32 timestamp_start = 0;

    timestamp_start = Delay_GetRuntimeUs();

    while (!ms5611_pressure_offset_flag)
    {
        MS5611_UpdateData();
        timestamp_now = Delay_GetRuntimeUs();
        // Timeout.
        if ((timestamp_now - timestamp_start) / 1000 >=
            MS5611_PRES_OFFSET_INIT_NUM * 50)
        {
            return 0;
        }
    }

    return 1;
}

// Read the converted result of MS5611.
u32 MS5611_GetConversion(void)
{
    u8  temp[3];
    u32 result = 0;

    IIC_SendStartSignal();
    IIC_WriteOneByte(MS5611_ADDR);
    IIC_WaitAckSignal();
    IIC_WriteOneByte(0);
    IIC_WaitAckSignal();
    IIC_SendStopSignal();

    IIC_SendStartSignal();
    // Enter receiving mode.
    IIC_WriteOneByte(MS5611_ADDR + 1);
    IIC_WaitAckSignal();
    // Read data with ACK  bit 23-16.
    temp[0] = IIC_ReadOneByte(1);
    // Read data with ACK  bit 8-15.
    temp[1] = IIC_ReadOneByte(1);
    // Read data with NACK bit 0-7.
    temp[2] = IIC_ReadOneByte(0);
    IIC_SendStopSignal();
    result = (u32)temp[0] * 65536 + (u32)temp[1] * 256 + (u32)temp[2];

    return result;
}

// Convert current pressure to height.
float MS5611_GetAltitude(void)
{
    float temp;
    float altitude;

    // Judge whether 0m pressure have been initialized or not.
    if (pressure_offset == 0)
    {
        // Use the average value of number of MS5611_PRES_OFFSET_INIT_NUM
        // calculation as height deviation.
        if (pressure_init_cnt > MS5611_PRES_OFFSET_INIT_NUM)
        {
            pressure_offset = pressure_offset_num / pressure_init_cnt;
            ms5611_pressure_offset_flag = true;
        }
        else
        {
            pressure_offset_num += ms5611_pressure;
        }
        pressure_init_cnt++;
        altitude = 0;
        return altitude;
    }

    temp     = 1 - pow((ms5611_pressure / pressure_offset), 0.1903);
    altitude = 4433000.0 * temp * 0.01F;
    altitude = altitude + altitude_offset ;

    return altitude;
}

float MS5611_GetAverage(float *buffer, u8 size)
{
    u8 i;
    float sum = 0.0;

    for (i = 0; i < size; i++)
    {
        sum += buffer[i];
    }

    return sum / size;
}
