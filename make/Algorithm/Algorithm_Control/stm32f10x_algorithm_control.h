/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_control.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.14
Description: declare the control function
Others:      none
Function List:
             1. void SetHeadFree(uint8_t on);
             2. static void PID_PostionCal(PID_Typedef * PID, float target,
                                           float measure, int32_t dertT);
             3. void ControlAttiAng(void);
             4. void ControlAttiRate(void);
             5. void ControlAlti(void);
             6. void ControlMotor(void);
             7. float EstimateMinThru(void);
             8. float EstimateHoverThru(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.11  modify the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_CONTROL_H__
#define __STM32F10X_ALGORITHM_CONTROL_H__

#include "stm32f10x.h"

#define  SLOW_THRO     200              /* Idle speed */
#define  ANGLE_MAX     40.0             /* The maximum angle of inclination */
#define  YAW_RATE_MAX  180.0f / M_PI_F  /* deg/s* /

/* Correct attitude error, it can counteract the initial unbalance caused by the offset of centre of gravity */
/* #define  Roll_error_init   7  If the breeze takes off and leaves to the left, Roll_error_init will increase positively,
                                 otherwise Roll_error_init will increase negatively */
/* #define  Pitch_error_init -5  If the breeze takes off and leaves to the ahead, Pitch_error_init will increase negatively,
                                 otherwise Pitch_error_init will increase positively */
/* Constant height */
#define LAND_SPEED   1.2f  /* m/s^2 */
#define ALT_VEL_MAX  4.0f

#define YAW_CORRECT

enum {CLIMB_RATE = 0, MANUAL, LANDING};

extern uint8_t zIntReset;
extern uint8_t isAltLimit;
extern uint8_t altCtrlMode;
extern uint8_t offLandFlag;

extern float   hoverThrust;
extern float   altLand;
extern float   thrustZSp;
extern float   thrustZInt;

/* PID structure */
typedef struct
{
    float P;
    float I;
    float D;
    float Desired;
    float Error;
    float PreError;
    float PrePreError;
    float Increment;
    float Integ;
    float iLimit;
    float Deriv;
    float Output;
}PID_Typedef;

/* Write or read flash parameter structure */
typedef struct
{
    u16 WriteBuf[10];
    u16 ReadBuf[10];
}Parameter_Typedef;

/* Sensor */
typedef struct int16_xyz
{
    int16_t X;
    int16_t Y;
    int16_t Z;
}S_INT16_XYZ;

typedef union 
{
    int16_t D[3];
    S_INT16_XYZ V;
}U_INT16_XYZ;

/* IMU */
typedef struct float_xyz
{
    float X;
    float Y;
    float Z;
}S_FLOAT_XYZ;

typedef union 
{
    float D[3];
    S_FLOAT_XYZ V;
}U_FLOAT_XYZ;

typedef struct float_angle
{
    float Roll;
    float Pitch;
    float Yaw;
}S_FLOAT_ANGLE;

extern S_FLOAT_XYZ ACC_F;
extern S_FLOAT_XYZ GYRO_F;           /* Conversion result ACC unit:G, GYRO unit: deg/s */
extern S_FLOAT_XYZ GYRO_I[3];        /* Gyro integrator */
extern S_FLOAT_XYZ DIF_ACC;          /* Differential acceleration */
extern S_FLOAT_XYZ EXP_ANGLE;        /* Expectation angle */
extern S_FLOAT_XYZ DIF_ANGLE;        /* The different between expectation and practical angle */
extern S_FLOAT_ANGLE Q_ANGLE;        /* The angle is calculated by using quaternion */
extern S_INT16_XYZ ACC_AVG;
extern S_INT16_XYZ GYRO_AVG;         /* The average value of ACC and processed gyro by using sliding-window filter */

extern u16 PIDWriteBuf[3];

extern PID_Typedef pitch_rate_PID;   /* The angular rate PID of pitch */
extern PID_Typedef pitch_angle_PID;  /* The angle PID of pitch */

extern PID_Typedef yaw_rate_PID;     /* The angular rate PID of yaw */
extern PID_Typedef yaw_angle_PID;    /* The angle PID of yaw */

extern PID_Typedef roll_rate_PID;    /* The angular rate PID of roll */
extern PID_Typedef roll_angle_PID;   /* The angle PID of roll */

extern PID_Typedef alt_PID;
extern PID_Typedef alt_vel_PID;

extern float gyroxGloble;
extern float gyroyGloble;

extern volatile unsigned char motorLock;

extern void SetHeadFree(uint8_t on);
static void PID_PostionCal(PID_Typedef * PID, float target, float measure, int32_t dertT);
extern void ControlAttiAng(void);
extern void ControlAttiRate(void);
extern void ControlAlti(void);
extern void ControlMotor(void);
extern float EstimateMinThru(void);
extern float EstimateHoverThru(void);

#endif
