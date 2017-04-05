/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_monitor.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.10.31
Description: implement the monitor function
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

#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_module_rpdata.h"
#include "stm32f10x_module_monitor.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_bar.h"
#include "stm32f10x_algorithm_control.h"

#define CONV_ENDIAN

uint8_t pcCmdFlag = 0;
//static uint8_t uploadBuf[32] = {0xAA, 0xAA};
#define PC_REQ_PID    0x02
#define PC_PID_PITCH  0x10
#define PC_PID_ROLL   0x11
#define PC_PID_YAW    0x12
#define PC_PID_ALT    0x14

/* Notice the address alignment, don't initialize 'sum' */
HawkerPacket_t up = {{0xAA, 0xAA}, 0x01, 18};      /* Upload packet */
DataPackage_t up2 = {{0xAA, 0xAA}, 0x02, 30, {0}};

/* Send packet */
static uint8_t sendPCBuf[64] = {0xAA, 0xAA, 0x01, 0x14, 0, 100, 0, 200, 0, 130
                                , 0, 0, 0, 100, 0, 0, 0, 200, 0, 0, 0, 30, 0, 10, 0x6B};

#define TEST_LEN 1 + 5
//uint8_t testData[0x0C+5]={0xAA,0xAF,0x10,0x0C,0,1,0,0,0,0,0,0,0,0,0,0,(uint8_t)(0xAA+0xAF+0x10+0x0C+0x01)};
uint8_t testData[1+5] = {0xAA, 0xAF, 0x02, 0x01, 0x01, (uint8_t)(0xAA + 0xAF + 0x02 + 1 + 1)};
static uint8_t sendCnt = 0;
static uint8_t checksum;

extern uint8_t gParamsSaveEEPROMRequest;

void CommPCTest(void)
{
    uint8_t i = 0;
    for (i = 0; i < TEST_LEN; i++)
    {
        CommPC(testData[i]);
    }
}

static void BufAdd8Chk(uint8_t a)
{
    //USART_WriteBuf(&UartTxbuf,_x);
    sendPCBuf[sendCnt++] = a;
    checksum += a;
}

static void BufAddInt16(int16_t a)
{
    BufAdd8Chk((uint8_t)(a>>8));
    BufAdd8Chk((uint8_t)(a&0xff));
}

static void BufAddArr(uint8_t* dat, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        BufAdd8Chk(dat[i]);
    }
}

static void BufUpload(void)
{
    USART_SendBuf(sendPCBuf, sendCnt);
    sendCnt  = 0;
    checksum = 0;
}

void CommPCUpload(uint8_t cmd)
{
    //USART_SendBuf(testData, 6);
    //sendPCBuf[0] = 0xAA;
    //sendPCBuf[1] = 0xAA;
    //sendPCBuf[2] = cmd;
    checksum = 0;
    //USART_ClearBuf(&UartTxbuf);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(cmd);

    switch (cmd)
    {
        case PC_PID_PITCH:
            BufAdd8Chk(0x0C);   /* Len */
            BufAddInt16((int16_t)((pitch_rate_PID.P  * 100)));
            BufAddInt16((int16_t)((pitch_rate_PID.I  * 100)));
            BufAddInt16((int16_t)((pitch_rate_PID.D  * 100)));
            BufAddInt16((int16_t)((pitch_angle_PID.P * 100)));
            BufAddInt16((int16_t)((pitch_angle_PID.I * 100)));
            BufAddInt16((int16_t)((pitch_angle_PID.D * 100)));
        break;

        case PC_PID_ROLL:
            BufAdd8Chk(0x0C);   /* Len */
            BufAddInt16((int16_t)((roll_rate_PID.P  * 100)));
            BufAddInt16((int16_t)((roll_rate_PID.I  * 100)));
            BufAddInt16((int16_t)((roll_rate_PID.D  * 100)));
            BufAddInt16((int16_t)((roll_angle_PID.P * 100)));
            BufAddInt16((int16_t)((roll_angle_PID.I * 100)));
            BufAddInt16((int16_t)((roll_angle_PID.D * 100)));
        break;

        case PC_PID_YAW:
            BufAdd8Chk(0x0C);   /* Len */
            BufAddInt16((int16_t)((yaw_rate_PID.P  * 100)));
            BufAddInt16((int16_t)((yaw_rate_PID.I  * 100)));
            BufAddInt16((int16_t)((yaw_rate_PID.D  * 100)));
            BufAddInt16((int16_t)((yaw_angle_PID.P * 100)));
            BufAddInt16((int16_t)((yaw_angle_PID.I * 100)));
            BufAddInt16((int16_t)((yaw_angle_PID.D * 100)));
        break;

        case PC_PID_ALT:
            BufAdd8Chk(0x0C);   /* Len */
            BufAddInt16((int16_t)((alt_vel_PID.P * 100)));
            BufAddInt16((int16_t)((alt_vel_PID.I * 100)));
            BufAddInt16((int16_t)((alt_vel_PID.D * 100)));
            BufAddInt16((int16_t)((alt_PID.P     * 100)));
            BufAddInt16((int16_t)((alt_PID.I     * 100)));
            BufAddInt16((int16_t)((alt_PID.D     * 100)));
        break;
    }
    BufAdd8Chk(checksum);
    BufUpload();
}

/* Receive */
#define DAT_MAX_LEN 32
static uint8_t cmd    = 0;
static uint8_t len    = 0;
static uint8_t chkSum = 0;
static uint8_t datCnt = 0;
static uint8_t datBuf[DAT_MAX_LEN] = {0};

enum{IDLE = 0, HEADER1, HEADER2, CMD, LEN, DATA, CHK};

static uint8_t ps = IDLE;

void CommPC(uint8_t c)
{
    switch (ps)
    {
        case IDLE:
            chkSum = 0;
            if (c == 0xAA)
            {
                ps = HEADER1;
            }
        break;

        case HEADER1:
            if (c == 0xAF)
            {
                ps = HEADER2;
            }
            else
            {
                ps = IDLE;
            }
        break;

        case HEADER2:
            cmd = c;
            ps  = CMD;
        break;

        case CMD:
            len = c;
            ps  = DATA;
            chkSum = 0xAA + 0xAF + len + cmd;
        break;

        case DATA:
            if (datCnt < len)
            {
                datBuf[datCnt++] = c;
                chkSum += c;
            }
            if (datCnt == len)
            {
                ps = CHK;
            }
        break;

        case CHK:
            if (chkSum == c)
            {
                pcCmdFlag = 1;        /* Specific cmd process executed in main, not in irq */
                //CommPCProcessCmd(); /* Process cmd */
                datCnt = 0;
            }
            ps = IDLE;
        break;
    }
}

void CommPCProcessCmd(void)
{
    //USART_ClearBuf(&UartTxbuf);   /* Prepare for sending */
    switch (cmd)
    {
        case PC_REQ_PID:
            if (datBuf[0] == 0x01)  /* Read PID */
            {
                CommPCUpload(PC_PID_PITCH);
                CommPCUpload(PC_PID_ROLL);
                CommPCUpload(PC_PID_YAW);
                CommPCUpload(PC_PID_ALT);
            }
        break;

        case PC_PID_PITCH:  /* Pitch sub pid, main pid */
            pitch_rate_PID.P  = (int16_t)(datBuf[0]<<8  | datBuf[1]) * 0.01f;
            pitch_rate_PID.I  = (int16_t)(datBuf[2]<<8  | datBuf[3]) * 0.01f;
            pitch_rate_PID.D  = (int16_t)(datBuf[4]<<8  | datBuf[5]) * 0.01f;
            pitch_angle_PID.P = (int16_t)(datBuf[6]<<8  | datBuf[7]) * 0.01f;
            pitch_angle_PID.I = (int16_t)(datBuf[8]<<8  | datBuf[9]) * 0.01f;
            pitch_angle_PID.D = (int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

            //CommPCUpload(PC_PID_PITCH);
            //USART_SendBuf(0xAA,1);
            ReturnPIDHead(PC_PID_PITCH);
            gParamsSaveEEPROMRequest = 1;
        break;

        case PC_PID_ROLL:   /* Roll sub pid, main pid */
            roll_rate_PID.P  = (int16_t)(datBuf[0]<<8  | datBuf[1]) * 0.01f;
            roll_rate_PID.I  = (int16_t)(datBuf[2]<<8  | datBuf[3]) * 0.01f;
            roll_rate_PID.D  = (int16_t)(datBuf[4]<<8  | datBuf[5]) * 0.01f;
            roll_angle_PID.P = (int16_t)(datBuf[6]<<8  | datBuf[7]) * 0.01f;
            roll_angle_PID.I = (int16_t)(datBuf[8]<<8  | datBuf[9]) * 0.01f;
            roll_angle_PID.D = (int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

            //CommPCUpload(PC_PID_ROLL);
            ReturnPIDHead(PC_PID_ROLL);
            gParamsSaveEEPROMRequest = 1;
        break;

        case PC_PID_YAW:    /* Yaw */
            yaw_rate_PID.P  = (int16_t)(datBuf[0]<<8  | datBuf[1]) * 0.01f;
            yaw_rate_PID.I  = (int16_t)(datBuf[2]<<8  | datBuf[3]) * 0.01f;
            yaw_rate_PID.D  = (int16_t)(datBuf[4]<<8  | datBuf[5]) * 0.01f;
            yaw_angle_PID.P = (int16_t)(datBuf[6]<<8  | datBuf[7]) * 0.01f;
            yaw_angle_PID.I = (int16_t)(datBuf[8]<<8  | datBuf[9]) * 0.01f;
            yaw_angle_PID.D = (int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

            //CommPCUpload(PC_PID_YAW);
            ReturnPIDHead(PC_PID_YAW);
            gParamsSaveEEPROMRequest = 1;
        break;

        case 0x13:
        break;

        case PC_PID_ALT: /* Alt sub pid, main pid */
            alt_vel_PID.P = (int16_t)(datBuf[0]<<8  | datBuf[1]) * 0.01f;
            alt_vel_PID.I = (int16_t)(datBuf[2]<<8  | datBuf[3]) * 0.01f;
            alt_vel_PID.D = (int16_t)(datBuf[4]<<8  | datBuf[5]) * 0.01f;
            alt_PID.P     = (int16_t)(datBuf[6]<<8  | datBuf[7]) * 0.01f;
            alt_PID.I     = (int16_t)(datBuf[8]<<8  | datBuf[9]) * 0.01f;
            alt_PID.D     = (int16_t)(datBuf[10]<<8 | datBuf[11]) * 0.01f;

            //CommPCUpload(PC_PID_ALT);
            ReturnPIDHead(PC_PID_ALT);
            gParamsSaveEEPROMRequest = 1;
        break;
    }
}

void ReturnPIDHead(uint8_t pidType)
{
    checksum = 0;
    sendCnt = 0;
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(pidType);
    BufAdd8Chk(0x0C);
    BufAddArr(datBuf, 12);
    BufAdd8Chk(checksum);
    BufUpload();
}

/* Interface with hawker */
void DebugUploadHandle(void)
{
    up.roll.val  = imu.roll  * 100;
    up.pitch.val = imu.pitch * 100;
    up.yaw.val   = imu.yaw   * 100;
    up.alti.val  = nav.z     * 100;  /* Combined */
    up.temp.val  = MS5611_Temperature * 100;
    up.pres.val  = MS5611_Pressure;
    up.speed.val = nav.vz * 100;

#ifdef CONV_ENDIAN
    EndianConvert(up.roll.b,  2);
    EndianConvert(up.pitch.b, 2);
    EndianConvert(up.yaw.b,   2);
    EndianConvert(up.alti.b,  4);
    EndianConvert(up.temp.b,  2);
    EndianConvert(up.pres.b,  4);
    EndianConvert(up.speed.b, 2);
#endif
}

/* Arm is high in front, convert to fit upper */
/* Notice: it can be improved!!! */
static void EndianConvert(uint8_t arr[], uint8_t len)
{
    uint8_t arrS[8], i;
    for (i = 0; i < len; i++)
    {
        arrS[i] = arr[i];
    }

    for (i = 0; i < len; i++)
    {
        arr[len-1-i] = arrS[i];
    }
}

static void DebugUploadHandle2(void)
{
    checksum = 0;
    //USART_ClearBuf(&UartTxbuf);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0xAA);
    BufAdd8Chk(0x02);  /* Cmd */
    BufAdd8Chk(30);    /* Len */

    /* Acc */
    BufAddInt16(imu.accb[0] * 1000);
    BufAddInt16(imu.accb[1] * 1000);
    BufAddInt16(imu.accb[2] * 1000);

    /* Gyro */
    BufAddInt16(imu.gyro[0] * 180.0f / M_PI_F * 100);
    BufAddInt16(imu.gyro[1] * 180.0f / M_PI_F * 100);
    BufAddInt16(imu.gyro[2] * 180.0f / M_PI_F * 100);

    /* Mag */
    BufAddInt16(0);
    BufAddInt16(0);
    BufAddInt16(0);

    /* Raw */
    BufAddInt16(imu.accRaw[0] * 1000);
    BufAddInt16(imu.accRaw[1] * 1000);
    BufAddInt16(imu.accRaw[2] * 1000);

    BufAddInt16(imu.gyroRaw[0] * 180.0f / M_PI_F * 100);
    BufAddInt16(imu.gyroRaw[1] * 180.0f / M_PI_F * 100);
    BufAddInt16(imu.gyroRaw[2] * 180.0f / M_PI_F * 100);

    BufAdd8Chk(checksum);
    BufUpload();
}

static void DebubUploadHandle3(void)
{
    uint8_t i;
    up2.cmd = 0x08;
    up2.len = 6 * 2;
    up2.data[0]  = 0;
    up2.data[1]  = 0;
    up2.data[2]  = 0;//((short)(MS5611_VerticalSpeed*1000))>>8;  /* Baro_speed */
    up2.data[3]  = 0;//((short)(MS5611_VerticalSpeed*1000))&0xff;
    up2.data[4]  = ((short)(-NRF_Data.pitch * 100))>>8;          /* Acc speed */
    up2.data[5]  = ((short)(-NRF_Data.pitch * 100))&0xff;        /* Pitch */
    up2.data[6]  = ((short)(MS5611_Altitude * 1000))>>8;
    up2.data[7]  = ((short)(MS5611_Altitude * 1000))&0xff;
    up2.data[8]  = ((short)(imu.accg[2] * 1000))>>8;             /* Accz */
    up2.data[9]  = ((short)(imu.accg[2] * 1000))&0xff;
    up2.data[10] = 0;                                            /* Inte alt of accz */
    up2.data[11] = 0;
    up2.sum = 0;

    for (i = 0; i < 4; i++)
    {
        up2.sum += *((uint8_t *)(&up2) + i);
    }

    for (i = 0; i < up2.len; i++)
    {
        up2.sum += up2.data[i];
    }

    USART_SendBuf((uint8_t *)(&up2), up2.len + 4);
    USART_SendBuf(&(up2.sum), 1);
}

void CommPCUploadHandle(void)
{
    static uint8_t pkgDivCnt = 0;
    uint8_t i = 0;
    pkgDivCnt++;

    if (pkgDivCnt > 2)
    {
        pkgDivCnt = 0;
    }
    if (pkgDivCnt == 0)  /* Div time to send different datapacket to avoid use too much cpu at a time */
    {
        DebugUploadHandle();
        for (i = 0; i < 10; i++)  /* Send data in ram address align */
        {
            sendPCBuf[i] = *((uint8_t *)(&up) + i);
        }

        for (i = 10; i < 16; i++)
        {
            sendPCBuf[i] = *((uint8_t *)(&up) + 2 + i);
        }

        for (i = 16; i < 23; i++)
        {
            sendPCBuf[i] = *((uint8_t *)(&up) + 4 + i);
        }
        sendPCBuf[22] = 0;

        for (i = 0; i < 22; i++)
        {
            sendPCBuf[22] += sendPCBuf[i];
        }
        USART_SendBuf(sendPCBuf, 23);
    }
    else if (pkgDivCnt == 1)
    {
        DebugUploadHandle2();
    }
    else if (pkgDivCnt == 2)
    {
        DebubUploadHandle3();
    }
}
