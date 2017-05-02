/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_imu.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.21
Description: Declare the imu function
Others:      none
Function List:
             1. void  IMU_ConvertEularToDCM(float dcm[3][3], float roll,
                                            float pitch, float yaw);
             2. void  IMU_GetSensorData(void);
             3. void  IMU_Init(void);
             4. void  IMU_InitNonLinearSO3AHRS(float acc_x, float acc_y,
                                               float acc_z, float mag_x,
                                               float mag_y, float mag_z);
             5. void  IMU_StartSO3Thread(void);
             6. void  IMU_UpdateNonLinearSO3AHRS(float gyr_x,  float gyr_y,
                                                 float gyr_z,  float acc_x,
                                                 float acc_y,  float acc_z,
                                                 float mag_x,  float mag_y,
                                                 float mag_z,  float two_kp,
                                                 float two_ki,
                                                 float delta_time);
             7. bool  IMU_Calibrate(void);
             8. bool  IMU_Check(void);
             9. float IMU_CalculateInverseSqrt(float number);
History:
<author>    <date>        <desc>
maksyuki    2017.01.11    Modify the module
myyerrol    2017.05.02    Format the module
*******************************************************************************/

#ifndef __STM32F10X_ALGORITHM_IMU_H__
#define __STM32F10X_ALGORITHM_IMU_H__

#include <stdbool.h>
#include "stm32f10x.h"

#define IMU_HIGH_FREQ_CTRL                      1

#ifdef  IMU_HIGH_FREQ_CTRL
#define IMU_SAMPLE_RATE                         200.0f
#else
#define IMU_SAMPLE_RATE                         100.0f
#endif

#define IMU_FILTER_CUTOFF_FREQ                  30.0f
// Calibrate time.
// Unit: ms.
#define IMU_CALI_TIME_ACC                       3000
// Unit: us.
#define IMU_CALI_TIME_GYR                       3000000l
// Unit: m/s^2.
#define IMU_CONSTANTS_ONE_G                     9.80665f
// Unit: kg/m^3.
#define IMU_CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C 1.225f
// Unit: j/(kg*k).
#define IMU_CONSTANTS_AIR_GAS_CONST             287.1f
// Unit: deg.
#define IMU_CONSTANTS_ABSOLUTE_NULL_CELSIUS    -273.15f
// Unit: m.
#define IMU_CONSTANTS_RADIUS_OF_EARTH           6371000

#define IMU_ROLL                                0
#define IMU_PITCH                               1
#define IMU_YAW                                 2
#define IMU_THRUST                              3

// Unit: m/s^2.
#define IMU_SENSOR_MAX_G                        8.0f
// Unit: deg/s
#define IMU_SENSOR_MAX_R                        2000.0f
#define IMU_ACC_SCALE                           (IMU_SENSOR_MAX_G / 32768.0f)
#define IMU_GYR_SCALE                           (IMU_SENSOR_MAX_R / 32768.0f)

// Unit: m/s^2.
#define IMU_ACCZ_ERR_MAX                        0.05
#define IMU_CHECK_COUNT                         5

#define IMU_SO3_COMP_PARAMS_KP                  1.0f
#define IMU_SO3_COMP_PARAMS_KI                  0.05f

// acc_b:  Acceleration in Body frame.
// acc_g:  Acceleration in Ground frame.
// dcm_gb: DCM(Direct cosine matrix) convert from Ground frame to Body frame.
typedef struct
{
    bool  flag_cali;
    bool  flag_ready;
    s16   acc_adc[3];
    s16   gyr_adc[3];
    s16   mag_adc[3];
    float acc_raw[3];
    float gyr_raw[3];
    float mag_raw[3];
    float acc_off[3];
    float gyr_off[3];
    float acc_b[3];
    float acc_g[3];
    float gyr[3];
    float dcm_gb[3][3];
    float quar[4];
    float roll_ang;
    float roll_rad;
    float pitch_ang;
    float pitch_rad;
    float yaw_ang;
    float yaw_rad;
} IMU_Table;

// dcm_bg: DCM(Direct cosine matrix) convert from Body frame to Ground frame.
// dcm_gb: DCM(Direct cosine matrix) convert from Ground frame to Body frame.
extern float dcm_bg[3][3];
extern float dcm_gb[3][3];
extern volatile float acc_filter[3];
extern volatile float gyr_filter[3];
extern bool imu_cali_flag;
extern IMU_Table IMU_TableStructure;

extern void  IMU_ConvertEularToDCM(float dcm[3][3], float roll, float pitch,
                                   float yaw);
extern void  IMU_GetSensorData(void);
extern void  IMU_Init(void);
extern void  IMU_InitNonLinearSO3AHRS(float acc_x, float acc_y, float acc_z,
                                      float mag_x, float mag_y, float mag_z);
extern void  IMU_StartSO3Thread(void);
extern void  IMU_UpdateNonLinearSO3AHRS(float gyr_x,  float gyr_y, float gyr_z,
                                        float acc_x,  float acc_y, float acc_z,
                                        float mag_x,  float mag_y, float mag_z,
                                        float two_kp, float two_ki,
                                        float delta_time);
extern bool  IMU_Calibrate(void);
extern bool  IMU_Check(void);
extern float IMU_CalculateInverseSqrt(float number);

#endif
