/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_bar.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.14
Description: implement the barometer function
Others:      none
Function List:
             1. static void Intertial_Filter_Predict(float dt, float x[3]);
             2. static void Intertial_Filter_Corrent(float e, float dt,
                                                     float x[3], int i, float w);
             3. void AltitudeCombineThread(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.11  modify the module
*******************************************************************************/

#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_algorithm_bar.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_it.h"

nav_t nav;       /* NED frame in earth */
float z_est[3];  /* Estimate z vz az */

uint8_t accUpdated = 0;

static float w_z_bar    = 0.5f;
static float w_z_acc    = 20.0f;
static float w_acc_bias = 0.05f;

/* Acceleration in NED frame */
float accel_NED[3] = {0.0f, 0.0f, -CONSTANTS_ONE_G};

float corr_bar = 0.0f;  /* m */

/* Store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
float corr_acc[3] = {0.0f, 0.0f, 0.0f};  /* NED m/s2 */
float acc_bias[3] = {0.0f, 0.0f, 0.0f};  /* Body frame */

/* Combine filter to correct err */
static void Interial_Filter_Predict(float dt, float x[3])
{
    x[0] += x[1] * dt + x[2] * dt * dt / 2.0f;
    x[1] += x[2] * dt;
}

static void Interial_Filter_Correct(float e, float dt, float x[3], int i, float w)
{
    float ewdt = e * w * dt;
    x[i] += ewdt;

    if (i == 0)
    {
        x[1] += w * ewdt;
        x[2] += w * w * ewdt / 3.0;
    }
    else if (i == 1)
    {
        x[2] += w * ewdt;
    }
}

/* TimeStamp in us. Thread should be executed every 2~20ms */
/* MS5611_Altitude, should be in m. (can be fixed to abs, not relative). positive above ground */
/* accFilted, should be filted */
/* To improve the accuracy of the barometer, fuse the data of accelerometer */
void AltitudeCombineThread(void)
{
    static uint32_t PreTime = 0;
    uint32_t NowTime;
    float dt;

    /* Accelerometer bias correction */
    float accel_bias_corr[3] = {0.0f, 0.0f, 0.0f};
    uint8_t i, j;

    NowTime = micros();
    dt = (PreTime > 0) ? ((NowTime - PreTime) / 1000000.0f) : 0;
    PreTime = NowTime;

    if (!PaOffsetInited)   /* Wait baro to init its offset */
    {
        return;
    }

    if (!imu.ready)
    {
        return;
    }

    if (Baro_Alt_Updated)  /* Store err when sensor update */
    {
        corr_bar = 0 - MS5611_Altitude - z_est[0];  /* MS5611_Altitude baro alt, is postive above offset level. not in NED. z_est is in NED frame. */
        Baro_Alt_Updated = 0;
    }

    if (accUpdated)
    {
        imu.accb[0] -= acc_bias[0];
        imu.accb[1] -= acc_bias[1];
        imu.accb[2] -= acc_bias[2];

        for (i = 0; i < 3; i++)
        {
            accel_NED[i] = 0.0f;
            for (j = 0; j < 3; j++)
            {
                accel_NED[i] += imu.DCMgb[j][i] * imu.accb[j];
            }
        }

        accel_NED[2] = -accel_NED[2];
        corr_acc[2]  = accel_NED[2] + CONSTANTS_ONE_G - z_est[2];
        accUpdated   = 0;
    }

    /* Correct accelerometer bias every time step */
    accel_bias_corr[2] -= corr_bar * w_z_bar * w_z_bar;

    /* Transform error vector from NED frame to body frame */
    for (i = 0; i < 3; i++)
    {
        float c = 0.0f;
        for (j = 0; j < 3; j++)
        {
            c += imu.DCMgb[i][j] * accel_bias_corr[j];
        }
        acc_bias[i] += c * w_acc_bias * dt;
    }

    acc_bias[2] = -acc_bias[2];

    /* Inertial filter prediction for altitude ??? */
    Interial_Filter_Predict(dt, z_est);
    Interial_Filter_Correct(corr_bar, dt, z_est, 0, w_z_bar);
    Interial_Filter_Correct(corr_acc[2], dt, z_est, 2, w_z_acc);

    nav.z  = z_est[0];
    nav.vz = z_est[1];
    nav.az = z_est[2];
}
