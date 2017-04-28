/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_comm_link.h
Author:      myyerrol
Version:     none
Create date: 2017.04.28
Description: Declare the communication operation function
Others:      none
Function List:
History:
<author>    <date>        <desc>
myyerrol    2017.04.28    Modify the module
*******************************************************************************/

#ifndef __STM32F10X_Module_COMM_LINK_H__
#define __STM32F10X_Module_COMM_LINK_H__

#include <stdbool.h>
#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_algorithm_control.h"

// Multiwii Serial Protocol(MSP).
#define COMM_LINK_MSP_SET_THRO       1
#define COMM_LINK_MSP_SET_YAW        2
#define COMM_LINK_MSP_SET_PITCH      3
#define COMM_LINK_MSP_SET_ROLL       4
#define COMM_LINK_MSP_ARM_IT         5
#define COMM_LINK_MSP_DISARM_IT      6
#define COMM_LINK_MSP_SET_4CON       7
#define COMM_LINK_MSP_SETOFF         8
#define COMM_LINK_MSP_LAND_DOWN      9
#define COMM_LINK_MSP_HOLD_ALT       10
#define COMM_LINK_MSP_STOP_HOLD_ALT  11
#define COMM_LINK_MSP_HEAD_FREE      12
#define COMM_LINK_MSP_STOP_HEAD_FREE 13
#define COMM_LINK_MSP_POS_HOLD       14
#define COMM_LINK_MSP_STOP_POS_HOLD  15
#define COMM_LINK_MSP_FLY_STATE      16
#define COMM_LINK_MSP_ACC_CALI       205

#define COMM_LINK_APP_DB_YAW         70
#define COMM_LINK_APP_DB_PR          50

#define COMM_LINK_STATE_EN_MCU        0
#define COMM_LINK_STATE_DISEN_MCU     1
#define COMM_LINK_STATE_REQ_EN_MCU    2
#define COMM_LINK_STATE_REQ_DISEN_MCU 3

#define COMM_LINK_CONSTRAIN(x, min, max) \
{                                        \
    if (x < min)                         \
    {                                    \
        x = min;                         \
    }                                    \
    if (x > max)                         \
    {                                    \
        x = max;                         \
    }                                    \
}

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
} CommLink_Data;

extern u8   comm_link_mcu_state;
extern u16  comm_link_rc_data[4];
extern bool comm_link_fly_enable_flag;
extern CommLink_Data CommLink_DataStructure;

extern float CommLink_CutDBScaleToLinear(float x_start, float x_end,
                                         float deadband);
extern void  CommLink_ProcessDataFromNRF(void);
extern void  CommLink_ReceiveDataFromNRF(void);

#endif
