/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_ms5611.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.01
Description: Declare the ms5611 function
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

#ifndef __STM32F10X_MODULE_MS5611_H__
#define __STM32F10X_MODULE_MS5611_H__

#include <stdbool.h>
#include "stm32f10x.h"

// Addresses of the device CSB = 0.
// Default IIC address.
#define MS5611_ADDR                     0XEE
// Registers of the device.
#define MS5611_D1                       0X40
#define MS5611_D2                       0X50
#define MS5611_RESET                    0X1E
// D1 and D2 result size(bytes).
#define MS5611_D1D2_SIZE                3

// OSR(Over Sampling Ratio)constants.
// Conversion time 0.6ms resolution 0.065mbar.
#define MS5611_OSR_256                  0X00
// Conversion time 1.2ms resolution 0.042mbar.
#define MS5611_OSR_512                  0X02
// Conversion time 2.3ms resolution 0.027mbar.
#define MS5611_OSR_1024                 0X04
// Conversion time 4.6ms resolution 0.018mbar.
#define MS5611_OSR_2048                 0X06
// Conversion time 9.1ms resolution 0.012mbar.
#define MS5611_OSR_4096                 0X08

// By adding ints from 0 to 6 we can read all the PROM configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2.
#define MS5611_PROM_BASE_ADDR           0XA2
// Number of registers in the PROM.
#define MS5611_PROM_REG_COUNT           6
// Size in bytes of a PROM registry.
#define MS5611_PROM_REG_SIZE            2

// Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar).
#define MS5611_MSLP                     101325

// The sampling precision of pressure.
#define MS5611_OSR_PRES                 MS5611_OSR_4096
// The sampling precision of temperature.
#define MS5611_OSR_TEMP                 MS5611_OSR_4096

#define MS5611_BUFFER_SIZE              10

// MS5611 state machine.
// [TEMP: Temperature] [PRES: Pressure].
#define MS5611_STATE_START_CONVERT_TEMP 0X01
#define MS5611_STATE_CONVERTING_TEMP    0X02
#define MS5611_STATE_START_CONVERT_PRES 0X03
#define MS5611_STATE_CONVERTING_PRES    0X04

#define MS5611_PRES_OFFSET_INIT_NUM     50

typedef int64_t s64;

extern bool ms5611_pressure_offset_flag;
extern bool ms5611_altitude_update_flag;

// Units: [temperature: c], [pressure: Pa], [altitude: m].
extern volatile float ms5611_altitude;
extern volatile float ms5611_pressure;
extern volatile float ms5611_temperature;

extern void  MS5611_AddNewAltitude(float value);
extern void  MS5611_AddNewTemperature(float value);
extern void  MS5611_AddNewPressure(float value);
extern void  MS5611_GetPressure(void);
extern void  MS5611_GetTemperature(void);
extern void  MS5611_Init(void);
extern void  MS5611_ReadPROM(void);
extern void  MS5611_Reset(void);
extern void  MS5611_StartConversion(u8 command);
extern void  MS5611_UpdateData(void);
extern u8    MS5611_WaitBaroInitOffset(void);
extern u32   MS5611_GetConversion(void);
extern float MS5611_GetAltitude(void);
extern float MS5611_GetAverage(float *buffer, u8 size);

#endif
