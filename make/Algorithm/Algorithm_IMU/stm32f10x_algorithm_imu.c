/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_imu.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.21
Description: Implement the IMU_TableStructure function
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

#include <math.h>
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_module_mpu6050.h"
#include "stm32f10x_algorithm_filter.h"
#include "stm32f10x_algorithm_imu.h"

bool imu_cali_flag = false;
IMU_Table IMU_TableStructure;

// Auxiliary variables to reduce number of repeated operations.
// Quaternion of sensor frame relative to auxiliary frame.
static float q0  = 1.0f;
static float q1  = 0.0f;
static float q2  = 0.0f;
static float q3  = 0.0f;
// Quaternion of sensor frame relative to auxiliary frame.
static float dq0 = 0.0f;
static float dq1 = 0.0f;
static float dq2 = 0.0f;
static float dq3 = 0.0f;

static float q0_q0, q0_q1, q0_q2, q0_q3;
static float q1_q1, q1_q2, q1_q3;
static float q2_q2, q2_q3;
static float q3_q3;
// Bias estimation.
static float gyr_bias[3] = {0.0f, 0.0f, 0.0f};
static bool  filter_init_flag = false;

// Convert DCM from Ground frame to Body frame.
void IMU_ConvertEularToDCM(float dcm[3][3], float roll, float pitch, float yaw)
{
    float sinx, siny, sinz, cosx, cosy, cosz;
    float cosz_cosx, cosz_cosy, sinz_cosx, cosz_sinx, sinz_sinx;

    sinx = sinf(roll *  M_PI / 180.0f);
    cosx = cosf(roll *  M_PI / 180.0f);
    siny = sinf(pitch * M_PI / 180.0f);
    cosy = cosf(pitch * M_PI / 180.0f);
    sinz = sinf(yaw *   M_PI / 180.0f);
    cosz = cosf(yaw *   M_PI / 180.0f);

    cosz_cosx = cosz * cosx;
    cosz_cosy = cosz * cosy;
    sinz_cosx = sinz * cosx;
    cosz_sinx = cosz * sinx;
    sinz_sinx = sinz * sinx;

    dcm[0][0] = cosz_cosy;
    dcm[0][1] = cosy * sinz;
    dcm[0][2] = -siny;
    dcm[1][0] = -sinz_cosx + (cosz_sinx * siny);
    dcm[1][1] = cosz_cosx + (sinz_sinx * siny);
    dcm[1][2] = sinx * cosy;
    dcm[2][0] = (sinz_sinx) + (cosz_cosx * siny);
    dcm[2][1] = -(cosz_sinx) + (sinz_cosx * siny);
    dcm[2][2] = cosy * cosx;
}

void IMU_GetSensorData(void)
{
    u8 i;

    MPU6050_ReadAcc(IMU_TableStructure.acc_adc);
    MPU6050_ReadGyr(IMU_TableStructure.gyr_adc);

    for (i = 0; i < 3; i++)
    {
        IMU_TableStructure.acc_raw[i] = (float) IMU_TableStructure.acc_adc[i] *
            IMU_ACC_SCALE * IMU_CONSTANTS_ONE_G;
        IMU_TableStructure.gyr_raw[i] = (float) IMU_TableStructure.gyr_adc[i] *
            IMU_GYR_SCALE * M_PI / 180.0f;
    }

    IMU_TableStructure.acc_b[0] = Filter_ApplyLPF2p_1(
        IMU_TableStructure.acc_raw[0] - IMU_TableStructure.acc_off[0]);
    IMU_TableStructure.acc_b[1] = Filter_ApplyLPF2p_2(
        IMU_TableStructure.acc_raw[1] - IMU_TableStructure.acc_off[1]);
    IMU_TableStructure.acc_b[2] = Filter_ApplyLPF2p_3(
        IMU_TableStructure.acc_raw[2] - IMU_TableStructure.acc_off[2]);
    IMU_TableStructure.gyr[0]   = Filter_ApplyLPF2p_4(
        IMU_TableStructure.gyr_raw[0]);
    IMU_TableStructure.gyr[1]   = Filter_ApplyLPF2p_5(
        IMU_TableStructure.gyr_raw[1]);
    IMU_TableStructure.gyr[2]   = Filter_ApplyLPF2p_6(
        IMU_TableStructure.gyr_raw[2]);
}

void IMU_Init(void)
{
    IMU_TableStructure.flag_ready = false;
    IMU_TableStructure.flag_cali  = true;

    Filter_SetLPF2pCutoffFreq_1(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    Filter_SetLPF2pCutoffFreq_2(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    Filter_SetLPF2pCutoffFreq_3(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    Filter_SetLPF2pCutoffFreq_4(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    Filter_SetLPF2pCutoffFreq_5(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    Filter_SetLPF2pCutoffFreq_6(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
}

// Using accelerometer and magnetometer to sense the gravity vector and yaw.
void  IMU_InitNonLinearSO3AHRS(float acc_x, float acc_y, float acc_z,
                               float mag_x, float mag_y, float mag_z)
{
    float init_roll, init_pitch;
    float cos_roll, sin_roll, cos_pitch, sin_pitch;
    float mag_x_temp, mag_y_temp;
    float init_hdg, cos_heading, sin_heading;

    init_roll  = atan2(-acc_y, -acc_z);
    init_pitch = atan2(acc_x, -acc_z);

    sin_roll  = sinf(init_roll);
    cos_roll  = cosf(init_roll);
    sin_pitch = sinf(init_pitch);
    cos_pitch = cosf(init_pitch);

    mag_x_temp = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch + mag_z *
        cos_roll * sin_pitch;
    mag_y_temp = mag_y * cos_roll - mag_z * sin_roll;

    init_hdg = atan2f(-mag_y_temp, mag_x_temp);

    cos_roll    = cosf(init_roll * 0.5f);
    sin_roll    = sinf(init_roll * 0.5f);

    cos_pitch   = cosf(init_pitch * 0.5f);
    sin_pitch   = sinf(init_pitch * 0.5f);

    cos_heading = cosf(init_hdg * 0.5f);
    sin_heading = sinf(init_hdg * 0.5f);

    q0 = cos_roll * cos_pitch * cos_heading + sin_roll * sin_pitch *
        sin_heading;
    q1 = sin_roll * cos_pitch * cos_heading - cos_roll * sin_pitch *
        sin_heading;
    q2 = cos_roll * sin_pitch * cos_heading + sin_roll * cos_pitch *
        sin_heading;
    q3 = cos_roll * cos_pitch * sin_heading - sin_roll * sin_pitch *
        cos_heading;

    q0_q0 = q0 * q0;
    q0_q1 = q0 * q1;
    q0_q2 = q0 * q2;
    q0_q3 = q0 * q3;
    q1_q1 = q1 * q1;
    q1_q2 = q1 * q2;
    q1_q3 = q1 * q3;
    q2_q2 = q2 * q2;
    q2_q3 = q2 * q3;
    q3_q3 = q3 * q3;
}

// Using software for attitude's calculation.
void IMU_StartSO3Thread(void)
{
    u8    i;
    u32   timestamp_now;
    // Unit: s.
    float delta_time    = 0.01f;
    // Output euler angles.
    // Unit: rad/s
    float euler[3]      = {0.0f, 0.0f, 0.0f};
    // Rotation matrix.
    float rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,
                           1.f};
    // Unit: m/s^2.
    float acc[3]        = {0.0f, 0.0f, 0.0f};
    float mag[3]        = {0.0f, 0.0f, 0.0f};
    // Unit: rad/s.
    float gyr[3]        = {0.0f, 0.0f, 0.0f};
    static u16 offset_count        = 0;
    // Unit: us.
    static u32 timestamp_pre       = 0;
    // Unit: us.
    static u32 timestamp_start     = 0;
    /* Need to calc gyr offset before IMU_TableStructure start working */
    static float gyr_offset_sum[3] = {0.0f, 0.0f, 0.0f};

    timestamp_now = Delay_GetRuntimeUs();
    delta_time    = (timestamp_pre > 0) ? (timestamp_now - timestamp_pre) /
        1000000.0f : 0;
    timestamp_pre = timestamp_now;

    IMU_GetSensorData();

    if (!IMU_TableStructure.flag_ready)
    {
        if (timestamp_start == 0)
        {
            timestamp_start = timestamp_now;
        }
        gyr_offset_sum[0] += IMU_TableStructure.gyr_raw[0];
        gyr_offset_sum[1] += IMU_TableStructure.gyr_raw[1];
        gyr_offset_sum[2] += IMU_TableStructure.gyr_raw[2];
        offset_count++;
        if (timestamp_now > timestamp_start + IMU_CALI_TIME_GYR)
        {
            IMU_TableStructure.gyr_off[0] = gyr_offset_sum[0] / offset_count;
            IMU_TableStructure.gyr_off[1] = gyr_offset_sum[1] / offset_count;
            IMU_TableStructure.gyr_off[2] = gyr_offset_sum[2] / offset_count;
            IMU_TableStructure.flag_ready = true;
            gyr_offset_sum[0] = 0;
            gyr_offset_sum[1] = 0;
            gyr_offset_sum[2] = 0;
            timestamp_start   = 0;
            offset_count      = 0;
        }
        return ;
    }

    gyr[0] = IMU_TableStructure.gyr[0] - IMU_TableStructure.gyr_off[0];
    gyr[1] = IMU_TableStructure.gyr[1] - IMU_TableStructure.gyr_off[1];
    gyr[2] = IMU_TableStructure.gyr[2] - IMU_TableStructure.gyr_off[2];

    acc[0] = -IMU_TableStructure.acc_b[0];
    acc[1] = -IMU_TableStructure.acc_b[1];
    acc[2] = -IMU_TableStructure.acc_b[2];

    IMU_UpdateNonLinearSO3AHRS(gyr[0],  gyr[1],  gyr[2],
                              -acc[0], -acc[1], -acc[2],
                               mag[0],  mag[1],  mag[2],
                               IMU_SO3_COMP_PARAMS_KP,
                               IMU_SO3_COMP_PARAMS_KI,
                               delta_time);

    // Convert q->R, This R converts Inertial frame to Body frame.
    rot_matrix[0] = q0_q0 + q1_q1 - q2_q2 - q3_q3;
    rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3);
    rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2);
    rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3);
    rot_matrix[4] = q0_q0 - q1_q1 + q2_q2 - q3_q3;
    rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1);
    rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2);
    rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1);
    rot_matrix[8] = q0_q0 - q1_q1 - q2_q2 + q3_q3;

    // Convert from Rotation matrix to Euler angles.
    // Roll.
    euler[0] =  atan2f(rot_matrix[5], rot_matrix[8]);
    // Pitch.
    euler[1] = -asinf(rot_matrix[2]);
    // Yaw.
    euler[2] =  atan2f(rot_matrix[1], rot_matrix[0]);

    /* dcm, ground to body */
    for (i = 0; i < 9; i++)
    {
        *(&(IMU_TableStructure.dcm_gb[0][0]) + i) = rot_matrix[i];
    }

    IMU_TableStructure.roll_rad  = euler[0];
    IMU_TableStructure.pitch_rad = euler[1];
    IMU_TableStructure.yaw_rad   = euler[2];

    IMU_TableStructure.roll_ang  = euler[0] * 180.0f / M_PI;
    IMU_TableStructure.pitch_ang = euler[1] * 180.0f / M_PI;
    IMU_TableStructure.yaw_ang   = euler[2] * 180.0f / M_PI;
}

// Using Mahony complementary filtering algorithm.
void  IMU_UpdateNonLinearSO3AHRS(float gyr_x,  float gyr_y,  float gyr_z,
                                 float acc_x,  float acc_y,  float acc_z,
                                 float mag_x,  float mag_y,  float mag_z,
                                 float two_kp, float two_ki, float delta_time)
{
    float recip_norm;
    float halfex = 0.0f;
    float halfey = 0.0f;
    float halfez = 0.0f;

    // Make filter converge to initial solution faster.
    // WARNING: in case air reboot, this can cause problem. But this is very
    // unlikely happen.
    if (!filter_init_flag)
    {
        IMU_InitNonLinearSO3AHRS(acc_x, acc_y, acc_z, mag_x, mag_y, mag_z);
        filter_init_flag = true;
    }

    //If magnetometer measurement is available, use it.
    if (!((mag_x == 0.0f) && (mag_y == 0.0f) && (mag_z == 0.0f)))
    {
        float hx, hy, hz, bx, bz;
        float halfwx, halfwy, halfwz;
        // Normalise magnetometer measurement.
        recip_norm = IMU_CalculateInverseSqrt(mag_x * mag_x +
                                              mag_y * mag_y +
                                              mag_z * mag_z);
        mag_x *= recip_norm;
        mag_y *= recip_norm;
        mag_z *= recip_norm;
        // Reference direction of Earth's magnetic field.
        hx = 2.0f * (mag_x * (0.5f - q2_q2 - q3_q3) + mag_y * (q1_q2 - q0_q3) +
            mag_z * (q1_q3 + q0_q2));
        hy = 2.0f * (mag_x * (q1_q2 + q0_q3) + mag_y * (0.5f - q1_q1 - q3_q3) +
            mag_z * (q2_q3 - q0_q1));
        hz = 2.0f *  mag_x * (q1_q3 - q0_q2) + 2.0f * mag_y * (q2_q3 + q0_q1) +
            2.0f * mag_z * (0.5f - q1_q1 - q2_q2);
        bx = sqrt(hx * hx + hy * hy);
        bz = hz;
        // Estimated direction of magnetic field.
        halfwx = bx * (0.5f - q2_q2 - q3_q3) + bz * (q1_q3 - q0_q2);
        halfwy = bx * (q1_q2 - q0_q3) + bz * (q0_q1 + q2_q3);
        halfwz = bx * (q0_q2 + q1_q3) + bz * (0.5f - q1_q1 - q2_q2);
        // Error is sum of cross product between estimated direction and
        // measured direction of field vectors.
        halfex += (mag_y * halfwz - mag_z * halfwy);
        halfey += (mag_z * halfwx - mag_x * halfwz);
        halfez += (mag_x * halfwy - mag_y * halfwx);
    }

    // Compute feedback only if accelerometer measurement valid(avoids NaN in
    // accelerometer normalisation).
    if (!((acc_x == 0.0f) && (acc_y == 0.0f) && (acc_z == 0.0f)))
    {
        float halfvx, halfvy, halfvz;
        // Normalise accelerometer measurement.
        recip_norm = IMU_CalculateInverseSqrt(acc_x * acc_x +
                                              acc_y * acc_y +
                                              acc_z * acc_z);
        acc_x *= recip_norm;
        acc_y *= recip_norm;
        acc_z *= recip_norm;
        // Estimated direction of gravity and magnetic field.
        halfvx = q1_q3 - q0_q2;
        halfvy = q0_q1 + q2_q3;
        halfvz = q0_q0 - 0.5f + q3_q3;
        // Error is sum of cross product between estimated direction and
        // measured direction of field vectors.
        halfex += acc_y * halfvz - acc_z * halfvy;
        halfey += acc_z * halfvx - acc_x * halfvz;
        halfez += acc_x * halfvy - acc_y * halfvx;
    }

    // Apply feedback only when valid data has been gathered from the
    // accelerometer or magnetometer.
    if ((halfex != 0.0f) && (halfey != 0.0f) && (halfez != 0.0f))
    {
        // Compute and apply integral feedback if enabled.
        if (two_ki > 0.0f)
        {
            // Integral error scaled by ki.
            gyr_bias[0] += two_ki * halfex * delta_time;
            gyr_bias[1] += two_ki * halfey * delta_time;
            gyr_bias[2] += two_ki * halfez * delta_time;
            // Apply integral feedback.
            gyr_x += gyr_bias[0];
            gyr_y += gyr_bias[1];
            gyr_z += gyr_bias[2];
        }
        else
        {
            // Prevent integral windup.
            gyr_bias[0] = 0.0f;
            gyr_bias[1] = 0.0f;
            gyr_bias[2] = 0.0f;
        }
        // Apply proportional feedback.
        gyr_x += two_kp * halfex;
        gyr_y += two_kp * halfey;
        gyr_z += two_kp * halfez;
    }

    // Time derivative of quaternion.
    dq0 = 0.5f * (-q1 * gyr_x - q2 * gyr_y - q3 * gyr_z);
    dq1 = 0.5f * ( q0 * gyr_x + q2 * gyr_z - q3 * gyr_y);
    dq2 = 0.5f * ( q0 * gyr_y - q1 * gyr_z + q3 * gyr_x);
    dq3 = 0.5f * ( q0 * gyr_z + q1 * gyr_y - q2 * gyr_x);

    q0 += delta_time * dq0;
    q1 += delta_time * dq1;
    q2 += delta_time * dq2;
    q3 += delta_time * dq3;

    // Normalise quaternion.
    recip_norm = IMU_CalculateInverseSqrt(q0 * q0 + q1 * q1 + q2 * q2 +
                                          q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;

    // Auxiliary variables to avoid repeated arithmetic.
    q0_q0 = q0 * q0;
    q0_q1 = q0 * q1;
    q0_q2 = q0 * q2;
    q0_q3 = q0 * q3;
    q1_q1 = q1 * q1;
    q1_q2 = q1 * q2;
    q1_q3 = q1 * q3;
    q2_q2 = q2 * q2;
    q2_q3 = q2 * q3;
    q3_q3 = q3 * q3;
}

bool IMU_Calibrate(void)
{
    u8 i                     = 0;
    u16 delta_time           = 0;
    u16 timestamp_now        = 0;
    bool return_flag         = false;
    static u16 count         = 0;
    static u16 timestamp_pre = 0;
    static bool cali_flag    = false;
    static float acc_sum[3]  = {0, 0, 0};
    static float gyr_sum[3]  = {0, 0, 0};

    timestamp_now = (u16)Delay_GetRuntimeMs();
    delta_time    = timestamp_now - timestamp_pre;

    if (!cali_flag)
    {
        cali_flag = true;
        for (i = 0; i < 3; i++)
        {
            count      = 0;
            acc_sum[i] = 0;
            gyr_sum[i] = 0;
            IMU_TableStructure.flag_ready = false;
        }
    }

    if (delta_time >= 10)
    {
        if (count < 300)
        {
            for (i = 0; i < 3; i++)
            {
                acc_sum[i] += IMU_TableStructure.acc_raw[i];
                gyr_sum[i] += IMU_TableStructure.gyr_raw[i];
            }
            count++;
            timestamp_pre = timestamp_now;
        }
        else
        {
            for (i = 0; i < 3; i++)
            {
                IMU_TableStructure.acc_off[i] = acc_sum[i] / (float)count;
                IMU_TableStructure.gyr_off[i] = gyr_sum[i] / (float)count;
            }
            IMU_TableStructure.acc_off[2] -= IMU_CONSTANTS_ONE_G;
            cali_flag   = false;
            return_flag = true;
        }
    }

    return return_flag;
}

bool IMU_Check(void)
{
    u8  i;
    u32 acc_z_sum    = 0;
    float acc_z_bias = 0;

    for (i = 0; i < IMU_CHECK_COUNT; i++)
    {
        MPU6050_ReadAcc(IMU_TableStructure.acc_adc);
        acc_z_sum += IMU_TableStructure.acc_adc[2];
    }

    IMU_TableStructure.acc_raw[2] = (float)(acc_z_sum / (float)IMU_CHECK_COUNT)
        * IMU_ACC_SCALE * IMU_CONSTANTS_ONE_G;
    acc_z_bias = IMU_TableStructure.acc_raw[2] - IMU_TableStructure.acc_off[2];

    if ((acc_z_bias > IMU_CONSTANTS_ONE_G - IMU_ACCZ_ERR_MAX) &&
        (acc_z_bias < IMU_CONSTANTS_ONE_G + IMU_ACCZ_ERR_MAX))
    {
        IMU_TableStructure.flag_cali = true;
    }
    else
    {
        IMU_TableStructure.flag_cali = false;
    }

    return IMU_TableStructure.flag_cali;
}

// Calculate inverse square-root by using Carmack algorithm.
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root.
float IMU_CalculateInverseSqrt(float number)
{
    volatile long  i;
    volatile float x;
    volatile float y;
    volatile const float f = 1.5f;

    x = number * 0.5f;
    y = number;
    i = *((long *)&y);
    i = 0X5F375A86 - (i >> 1);
    y = *((float *)&i);
    y = y * (f - (x * y * y));

    return y;
}
