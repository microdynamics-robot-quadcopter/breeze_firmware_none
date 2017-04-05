/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_ms5611.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.01
Description: declare the ms5611 function
Others:      none
Function List:
             1. void MS5611_Reset(void);
             2. void MS5611_ReadPROM(void);
             3. void MS5611_TempPush(float val);
             4. void MS5611_PressPush(float val);
             5. void MS5611_AltPush(float val);
             6. float MS5611_GetAvg(float *buff, int size);
             7. void MS5611_StartConversion(uint8_t cmd);
             8. uint32_t MS5611_GetConversion(void);
             9. void MS5611_GetPressure(void);
            10. float MS5611_GetAltitude(void);
            11. void MS5611_GetTemperature(void);
            12. void MS5611_Init(void);
            13. void MS5611_Thread(void);
            14. uint8_t MS5611_WaitBaroInitOffset(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.01  modify the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_MS5611_H__
#define __STM32F10X_MODULE_MS5611_H__

#include "stm32f10x.h"

/* Addresses of the device CSB = 0 */
#define MS5611_ADDR           0xEE    /* Default IIC address */
#define MS5611_D1             0x40    /* Registers of the device */
#define MS5611_D2             0x50
#define MS5611_RESET          0x1E

#define MS5611_D1D2_SIZE      3       /* D1 and D2 result size(bytes) */

/* OSR(Over Sampling Ratio)constants */
#define MS5611_OSR_256        0x00    /* Conversion time 0.6ms resolution 0.065mbar */
#define MS5611_OSR_512        0x02    /* Conversion time 1.2ms resolution 0.042mbar */
#define MS5611_OSR_1024       0x04    /* Conversion time 2.3ms resolution 0.027mbar */
#define MS5611_OSR_2048       0x06    /* Conversion time 4.6ms resolution 0.018mbar */
#define MS5611_OSR_4096       0x08    /* Conversion time 9.1ms resolution 0.012mbar */

/* By adding ints from 0 to 6 we can read all the PROM configuration values */
/* C1 will be at 0xA2 and all the subsequent are multiples of 2 */
#define MS5611_PROM_BASE_ADDR 0xA2
#define MS5611_PROM_REG_COUNT 6       /* Number of registers in the PROM */
#define MS5611_PROM_REG_SIZE  2       /* Size in bytes of a PROM registry */

#define MSLP                  101325  /*mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)*/

/* Temperature in 1C            */
/* Pressure    in 0.01mbar = Pa */
/* Altitude    in m             */
extern volatile float MS5611_Pressure;
extern volatile float MS5611_Altitude;
extern volatile float MS5611_Temperature;
extern uint8_t PaOffsetInited;
extern uint8_t Baro_Alt_Updated;  /* The flag of completing height update */

extern void MS5611_Reset(void);
extern void MS5611_ReadPROM(void);
extern void MS5611_TempPush(float val);
extern void MS5611_PressPush(float val);
extern void MS5611_AltPush(float val);

extern float MS5611_GetAvg(float *buff, int size);
extern void MS5611_StartConversion(uint8_t cmd);
extern uint32_t MS5611_GetConversion(void);

extern void MS5611_GetPressure(void);
extern float MS5611_GetAltitude(void);
extern void MS5611_GetTemperature(void);

extern void MS5611_Init(void);
extern void MS5611_Thread(void);
extern uint8_t MS5611_WaitBaroInitOffset(void);

#endif
