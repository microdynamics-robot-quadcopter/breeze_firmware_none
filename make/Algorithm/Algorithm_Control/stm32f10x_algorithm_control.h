/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_control.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.14
Description: Declare the control function
Others:      none
Function List:
History:
<author>    <date>        <desc>
maksyuki    2017.01.11    Modify the module
myyerrol    2017.04.30    Format the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_CONTROL_H__
#define __STM32F10X_ALGORITHM_CONTROL_H__

#include <stdbool.h>
#include <math.h>
#include "stm32f10x.h"

#define CONTROL_THRUST_SLOW      200
// The maximum angle of inclination.
#define CONTROL_ANGLE_MAX        40.0
#define CONTROL_YAW_RATE_MAX     180.0f / M_PI
// Set fixed height.
#define CONTROL_LAND_SPEED       1.2f
#define CONTROL_ALT_VEL_MAX      4.0f
#define CONTROL_YAW_CORRECT      1

#define CONTROL_STATE_CLIMB_RATE 0
#define CONTROL_STATE_MANUAL     1
#define CONTROL_STATE_LANDING    2

#define CONTROL_ALT_FEED_FORWARD 0.5f
#define CONTROL_THRUST_MAX       1.0f
// Limited height is 3.5m.
#define CONTROL_ALT_LIMIT        2.0f
#define CONTROL_TILT_MAX         (CONTROL_ANGLE_MAX * M_PI / 180.0)
// Z deadband.
#define CONTROL_ALT_CTRL_Z_DB    1.0f

// PID infromation.
// kp: proportional's factor.
// ki: integral's factor.
// kd: derivative's factor.
// error: error.
// error_pre: previous error.
// error_der: error's derivative.
// error_int: error's integral.
// limit_int: integral's limit.
// output: PID's output.
typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float error_pre;
    float error_der;
    float error_int;
    float limit_int;
    float output;
} Control_PID;

typedef struct
{
    s16 x;
    s16 y;
    s16 z;
} Control_XYZInt16;

typedef struct
{
    float x;
    float y;
    float z;
} Control_XYZFloat;

typedef struct
{
    float r;
    float p;
    float y;
} Control_RPYAngle;

extern u8    control_alt_control_mode;
extern bool  control_integral_reset_flag;
extern bool  control_offland_flag;
extern float control_hover_thrust;
extern float control_alt_land;
extern float control_thrust_z_split_power;
extern float control_thrust_z_integral;

// Differential acceleration.
extern Control_XYZFloat Control_XYZFloatDiffAcc;
// Quaternion's angle.
extern Control_RPYAngle Control_RPYAngleQuaternion;
// The angle PID of pitch.
extern Control_PID Control_PIDPitchAngle;
// The angular rate PID of pitch.
extern Control_PID Control_PIDPitchAngleRate;
// The angle PID of yaw.
extern Control_PID Control_PIDYawAngle;
// The angular rate PID of yaw.
extern Control_PID Control_PIDYawAngleRate;
// The angle PID of roll.
extern Control_PID Control_PIDRollAngle;
// The angular rate PID of roll.
extern Control_PID Control_PIDRollAngleRate;
extern Control_PID Control_PIDAlt;
extern Control_PID Control_PIDAltVel;

extern void  Control_CallPIDAngle(void);
extern void  Control_CallPIDAngleRate(void);
extern void  Control_CallPIDPostion(Control_PID *pid, float target,
                                    float measure, s32 delta_time);
extern void  Control_SetAltitude(void);
extern void  Control_SetHeadFreeMode(bool flag);
extern void  Control_SetMotorPWM(void);
extern float Control_EstimateThrustRefMin(void);
extern float Control_EstimateThrustRefHover(void);

#endif
