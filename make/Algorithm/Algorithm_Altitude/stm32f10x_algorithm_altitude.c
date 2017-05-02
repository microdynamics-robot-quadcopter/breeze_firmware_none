/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_altitude.c
Author:      myyerrol
Version:     none
Create date: 2017.04.30
Description: Implement the altitude function
Others:      none
Function List:
             1. void Altitude_CombineData(void);
             2. void Altitude_CorrectByUsingInertialFilter(float delta_time,
                                                           float data[3],
                                                           float weight,
                                                           float corr_factor,
                                                           u8 i);
             3. void Altitude_PredictByUsingInertialFilter(float delta_time,
                                              float data[3]);
History:
<author>    <date>        <desc>
myyerrol    2017.04.30    Modify the module
*******************************************************************************/

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_algorithm_altitude.h"
#include "stm32f10x_algorithm_imu.h"

bool altitude_acc_update_flag = false;
Altitude_NEDFrame Altitude_NEDFrameStructure;

// Estimate: pos_z, vel_z, acc_z.
static float z_estimate[3];
static float weight_z_bar    = 0.5f;
static float weight_z_acc    = 20.0f;
static float weight_acc_bias = 0.05f;
// Acceleration's data in NED frame.
static float ned_acc_data[3] = {0.0f, 0.0f, -IMU_CONSTANTS_ONE_G};
// Barometer's correctional factor.
// Unit: m.
static float bar_corr_factor = 0.0f;
// Acceleration's correctional data in NED frame.
static float ned_acc_corr[3] = {0.0f, 0.0f, 0.0f};
// Acceleration's bias in Body frame.
static float body_acc_bias[3] = {0.0f, 0.0f, 0.0f};

void Altitude_CombineData(void)
{
    u8 i, j;
    static u32 timestamp_pre = 0;
           u32 timestamp_now = 0;
    float delta_time;
    float acc_bias_corr[3]   = {0.0f, 0.0f, 0.0f};

    timestamp_now = Delay_GetRuntimeUs();
    delta_time    = (timestamp_pre > 0) ?
        ((timestamp_now - timestamp_pre) / 1000000.0f) : 0;
    timestamp_pre = timestamp_now;

    if (!ms5611_pressure_offset_flag)
    {
        return ;
    }

    if (!IMU_TableStructure.flag_ready)
    {
        return ;
    }

    if (ms5611_altitude_update_flag)
    {
        // The ms5611_altitude is postive above offset level.
        bar_corr_factor = 0 - ms5611_altitude - z_estimate[0];
        ms5611_altitude_update_flag = false;
    }

    if (altitude_acc_update_flag)
    {
        IMU_TableStructure.acc_b[0] -= body_acc_bias[0];
        IMU_TableStructure.acc_b[1] -= body_acc_bias[1];
        IMU_TableStructure.acc_b[2] -= body_acc_bias[2];

        for (i = 0; i < 3; i++)
        {
            ned_acc_data[i] = 0.0f;
            for (j = 0; j < 3; j++)
            {
                ned_acc_data[i] += IMU_TableStructure.dcm_gb[j][i] *
                    IMU_TableStructure.acc_b[j];
            }
        }

        ned_acc_data[2] = -ned_acc_data[2];
        ned_acc_corr[2] =  ned_acc_data[2] + IMU_CONSTANTS_ONE_G - z_estimate[2];
        altitude_acc_update_flag = false;
    }

    // Correct accelerometer's bias every time step.
    acc_bias_corr[2] -= bar_corr_factor * weight_z_bar * weight_z_bar;

    // Transform bias vector from NED frame to Body frame.
    for (i = 0; i < 3; i++)
    {
        float temp = 0.0f;
        for (j = 0; j < 3; j++)
        {
            temp += IMU_TableStructure.dcm_gb[i][j] * acc_bias_corr[j];
        }
        body_acc_bias[i] += temp * weight_acc_bias * delta_time;
    }

    body_acc_bias[2] = -body_acc_bias[2];

    Altitude_PredictByUsingInertialFilter(delta_time, z_estimate);
    Altitude_CorrectByUsingInertialFilter(delta_time, z_estimate, weight_z_bar,
                                          bar_corr_factor, 0);
    Altitude_CorrectByUsingInertialFilter(delta_time, z_estimate, weight_z_acc,
                                          ned_acc_corr[2], 2);

    Altitude_NEDFrameStructure.pos_z = z_estimate[0];
    Altitude_NEDFrameStructure.vel_z = z_estimate[1];
    Altitude_NEDFrameStructure.acc_z = z_estimate[2];
}

void Altitude_CorrectByUsingInertialFilter(float delta_time, float data[3],
                                           float weight, float corr_factor,
                                           u8 i)
{
    float temp = corr_factor * weight * delta_time;
    data[i]   += temp;

    if (i == 0)
    {
        data[1] += weight * temp;
        data[2] += weight * weight * temp / 3.0;
    }
    else if (i == 1)
    {
        data[2] += weight * temp;
    }
}

void Altitude_PredictByUsingInertialFilter(float delta_time, float data[3])
{
    data[0] = data[1] * delta_time + data[2] * delta_time * delta_time / 2.0f;
    data[1] = data[2] * delta_time;
}
