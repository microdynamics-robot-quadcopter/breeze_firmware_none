/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_imu.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.21
Description: declare the imu function
Others:      none
Function List:
             1. void IMU_Init(void);
             2. void IMU_Process(void);
             3. uint8_t IMU_Check(void);
             4. uint8_t IMU_Calibrate(void);
             5. void IMU_ReadSensorHandle(void);
             6. static void EularToDCM(float DCM[3][3], float pitch,
                                       float yaw, float roll);
             7. static float InvSqrt(float num);
             8. static void NonLinearSO3AHRSInit(float ax, float ay, float az,
                                                 float mx, float my, float mz);
             9. static void NonLinearSO3AHRSUpdate(float gx, float gy, float gz,
                                                   float ax, float ay, float az,
                                                   float mx, float my, float mz,
                                                   float twoKp, float twoKi,
                                                   float dt);
            10. void IMU_SO3Thread(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.11  modify the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_IMU_H__
#define __STM32F10X_ALGORITHM_IMU_H__

#include "stm32f10x.h"

#define HIGH_FREQ_CTRL                    /* Important!!! */

#ifdef  HIGH_FREQ_CTRL
#define IMU_SAMPLE_RATE         200.0f
#else
#define IMU_SAMPLE_RATE         100.0f    /* 1000.0f/(float)DMP_CALC_PRD */
#endif

#define IMU_FILTER_CUTOFF_FREQ	30.0f

/* Calibrate time */
#define ACC_CALC_TIME           3000      /* ms */
#define GYRO_CALC_TIME          3000000l  /* us */

typedef float  quad[4];
typedef float  vector3f[3];               /* Cannot be return value, pointer */
typedef float  matrix3f[3][3];

typedef struct mat3_tt
{
    float m[3][3];
}mat3;

typedef struct vec3_tt
{
    float v[3];
}vec3;

enum{ROLL, PITCH, YAW, THROTTLE};
enum {X, Y, Z};

typedef struct
{
    uint8_t caliPass;
    uint8_t ready;
    int16_t accADC[3];
    int16_t gyroADC[3];
    int16_t magADC[3];
    float   accRaw[3];      /* m/s^2 */
    float   magRaw[3];
    float   gyroRaw[3];     /* rad/s */
    float   accOffset[3];   /* m/s^2 */
    float   gyroOffset[3];
    float   accb[3];        /* Filted, in body frame */
    float   accg[3];
    float   gyro[3];
    float   DCMgb[3][3];
    float   q[4];
    float   yaw;
    float   roll;           /* deg */
    float   pitch;
    float   yawRad;
    float   rollRad;        /* rad */
    float   pitchRad;
}imu_t;

#define M_PI_F                               3.1415926
#define CONSTANTS_ONE_G                      9.80665f   /* m/s^2 */
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C  1.225f     /* kg/m^3 */
#define CONSTANTS_AIR_GAS_CONST              287.1f     /* j/(kg*k) */
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS      -273.15f   /* deg */
#define CONSTANTS_RADIUS_OF_EARTH            6371000    /* m */

extern volatile float AccFilted[3], GyroFilted[3];
extern float DCMbg[3][3], DCMgb[3][3];
extern float AccZOffsetTemp;

extern float IMU_Pitch;
extern float IMU_Roll;
extern float IMU_Yaw;

extern imu_t imu;
extern uint8_t imuCaliFlag;

extern void IMU_Init(void);
extern void IMU_Process(void);
extern uint8_t IMU_Check(void);
extern uint8_t IMU_Calibrate(void);
extern void IMU_ReadSensorHandle(void);

static void EularToDCM(float DCM[3][3], float pitch, float yaw, float roll);  /* DMP needs */
static float InvSqrt(float num);

static void NonLinearSO3AHRSInit(float ax, float ay, float az, float mx, float my, float mz);
static void NonLinearSO3AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);
extern void IMU_SO3Thread(void);

#endif
