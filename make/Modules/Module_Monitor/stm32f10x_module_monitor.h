/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_monitor.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.10.31
Description: declare the monitor function
Others:      none
Function List:
             1. void CommPC(uint8_t c);
             2. void CommPCTest(void);
             3. void CommPCUpload(uint8_t cmd);
             4. void CommPCProcessCmd(void);
             5. void ReturnPIDHead(uint8_t pidType);
             6. void CommPCUploadHandle(void);
             7. void DebugUploadHandle(void);
             8. static void DebugUploadHandle2(void);
             9. static void DebubUploadHandle3(void);
            10. static void EndianConvert(uint8_t arr[], uint8_t len);
            11. static void BufUpload(void);
            12. static void BufAddArr(uint8_t* dat, uint8_t len);
            13. static void BufAddInt16(int16_t a);
            14. static void BufAdd8Chk(uint8_t a);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.31  modify the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_MONITOR__
#define __STM32F10X_MODULE_MONITOR__

#include "stm32f10x.h"

#define UartSendInt16(_x) UartSendBuffer((uint8_t *)(&_x), 2)
#define UartSendInt32(_x) UartSendBuffer((uint8_t *)(&_x), 4)

/* Only send data */
typedef union int16un
{
    uint8_t b[2];
    int16_t val;
}int16_un;

typedef union int32un
{
    uint8_t b[4];
    int32_t val;
}int32_un;

typedef struct HawkerPacket_tt
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t len;
    uint8_t sum;

    int16_un roll;
    int16_un pitch;
    int16_un yaw;
    int32_un alti;
    int16_un temp;
    int32_un pres;
    int16_un speed;
}HawkerPacket_t;

typedef struct DataPackage_tt
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t len;
    uint8_t data[30];
    uint8_t sum;
}DataPackage_t;

extern HawkerPacket_t up;
extern uint8_t pcCmdFlag;
extern uint16_t rcData[4];

extern void CommPC(uint8_t c);
extern void CommPCTest(void);
extern void CommPCUpload(uint8_t cmd);
extern void CommPCProcessCmd(void);
extern void ReturnPIDHead(uint8_t pidType);
extern void CommPCUploadHandle(void);
extern void DebugUploadHandle(void);
static void DebugUploadHandle2(void);
static void DebubUploadHandle3(void);
static void EndianConvert(uint8_t arr[], uint8_t len);
static void BufUpload(void);
static void BufAddArr(uint8_t* dat, uint8_t len);
static void BufAddInt16(int16_t a);
static void BufAdd8Chk(uint8_t a);

#endif
