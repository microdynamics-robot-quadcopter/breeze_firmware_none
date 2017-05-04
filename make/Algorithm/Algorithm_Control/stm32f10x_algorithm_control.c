/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_control.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.14
Description: Implement the control function
Others:      none
Function List:
             1. void  Control_CallPIDAngle(void);
             2. void  Control_CallPIDAngleRate(void);
             3. void  Control_CallPIDPosition(Control_PID *pid, float target,
                                              float measure, s32 delta_time);
             4. void  Control_SetAltitude(void);
             5. void  Control_SetHeadFreeMode(bool flag);
             6. void  Control_SetMotorPWM(void);
             7. float Control_EstimateThrustRefMin(void);
             8. float Control_EstimateThrustRefHover(void);
History:
<author>    <date>        <desc>
maksyuki    2017.01.11    Modify the module
myyerrol    2017.04.30    Format the module
*******************************************************************************/

#include "stm32f10x_it.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_motor.h"
#include "stm32f10x_algorithm_altitude.h"
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_algorithm_imu.h"

u8    control_altitude_mode;
float control_alt_land;
float control_thrust_z_integral    = 0;
float control_thrust_z_split_power = 0;
bool  control_offland_flag         = false;
bool  control_integral_reset_flag  = true;

Control_PID Control_PIDPitchAngle;
Control_PID Control_PIDPitchAngleRate;
Control_PID Control_PIDRollAngle;
Control_PID Control_PIDRollAngleRate;
Control_PID Control_PIDYawAngle;
Control_PID Control_PIDYawAngleRate;
Control_PID Control_PIDAlt;
Control_PID Control_PIDAltVel;

static s16 output_motor[4] = {0};
// According to split power, calculate the expected roll and pitch.
static float split_power_roll  = 0;
static float split_power_pitch = 0;
static float output_roll       = 0;
static float output_pitch      = 0;
static float output_yaw        = 0;
static float output_thrust     = 0;
static float head_yaw_angle    = 0;
static float split_power_z_move_rate;
// Output roll and pitch.
static float thrust_xy_split_power[2] = {0, 0};
static bool head_free_mode_flag = false;
static bool altitude_limit_flag = false;
static bool saturation_z_flag   = false;
static bool saturation_xy_flag  = false;

// Control quadcopter's attitude. Angle control in cascade pid.
void Control_CallPIDAngle(void)
{
    float delta_time         = 0;
    float timestamp_now      = 0;
    float target_angle[3]    = {0, 0, 0};
    static u32 timestamp_pre = 0;

    timestamp_now = Delay_GetRuntimeUs();
    delta_time    = (timestamp_pre > 0) ? (timestamp_now - timestamp_pre) : 0;
    timestamp_pre = timestamp_now;

    if (control_altitude_mode == CONTROL_STATE_MANUAL)
    {
        target_angle[IMU_ROLL]  = (float)CommLink_DataStructure.roll;
        target_angle[IMU_PITCH] = (float)CommLink_DataStructure.pitch;
    }
    else
    {
        target_angle[IMU_ROLL]  = split_power_roll;
        target_angle[IMU_PITCH] = split_power_pitch;
    }

    if (head_free_mode_flag)
    {
#if CONTROL_YAW_CORRECT
        float diff_rad      = -(IMU_TableStructure.yaw_ang - head_yaw_angle) *
            M_PI / 180.0f;
#else
        float diff_rad      =  (IMU_TableStructure.yaw_ang - head_yaw_angle) *
            M_PI / 180.0f;
#endif
        float diff_cos      = cosf(diff_rad);
        float diff_sin      = sinf(diff_rad);
        target_angle[IMU_ROLL]  = target_angle[IMU_ROLL]  * diff_cos -
            target_angle[IMU_PITCH] * diff_sin;
        target_angle[IMU_PITCH] = target_angle[IMU_PITCH] * diff_cos +
            target_angle[IMU_ROLL]  * diff_sin;
    }

    Control_CallPIDPosition(&Control_PIDPitchAngle, target_angle[IMU_PITCH],
                            IMU_TableStructure.pitch_ang, delta_time);
    Control_CallPIDPosition(&Control_PIDRollAngle, target_angle[IMU_ROLL],
                            IMU_TableStructure.roll_ang, delta_time);
}

// Control quadcopter's attitude. Angle rate control in cascade pid.
void Control_CallPIDAngleRate(void)
{
    float delta_time         = 0;
    float timestamp_now      = 0;
    float yaw_angle_rate     = 0;
    static u32 timestamp_pre = 0;

    timestamp_now = Delay_GetRuntimeUs();
    delta_time    = (timestamp_pre > 0) ? (timestamp_now - timestamp_pre) : 0;
    timestamp_pre = timestamp_now;

    yaw_angle_rate = -(float)CommLink_DataStructure.yaw;

    // Note: original pid parameters are in AD value, need to convert.
    Control_CallPIDPosition(&Control_PIDPitchAngleRate,
                            Control_PIDPitchAngle.output,
                            IMU_TableStructure.gyr[IMU_PITCH] * 180.0f / M_PI,
                            delta_time);
    Control_CallPIDPosition(&Control_PIDRollAngleRate,
                            Control_PIDRollAngle.output,
                            IMU_TableStructure.gyr[IMU_ROLL]  * 180.0f / M_PI,
                            delta_time);
    Control_CallPIDPosition(&Control_PIDYawAngleRate,
                            yaw_angle_rate,
                            IMU_TableStructure.gyr[IMU_YAW]   * 180.0f / M_PI,
                            delta_time);

    output_pitch = Control_PIDPitchAngleRate.output;
    output_roll  = Control_PIDRollAngleRate.output;
    output_yaw   = Control_PIDYawAngleRate.output;
}

void Control_CallPIDPosition(Control_PID *pid, float target, float measure,
                             s32 delta_time)
{
    float temp_integral   = 0;
    float temp_delta_time = delta_time / 1000000.0;

    pid->error     = target - measure;
    pid->error_der = (pid->error - pid->error_pre) / temp_delta_time;
    pid->output    = (pid->kp * pid->error) + (pid->ki * pid->error_int) +
        (pid->kd * pid->error_der);
    pid->error_pre = pid->error;

    // Only use in angle and angular rate.
    if (comm_link_fly_enable_flag && control_offland_flag)
    {
        // Don't integrate when output is larger than thrust.
        if (fabs(pid->output) < output_thrust)
        {
            temp_integral = (pid->error_int) + (pid->error) * temp_delta_time;
            // Integrate in -300~300 range.
            if ((temp_integral > -pid->limit_int) &&
                (temp_integral <  pid->limit_int) &&
                (pid->output > -pid->limit_int)   &&
                (pid->output <  pid->limit_int))
            {
                pid->error_int = temp_integral;
            }
        }
    }
    else
    {
        pid->error_int = 0;
    }
}

// Control fixed height.
void Control_SetAltitude(void)
{
    float alt                        = 0;
    float alt_split_power            = 0;
    float alt_split_power_offset     = 0;
    float alt_split_power_offset_max = 0;
    float pos_z_error                = 0;
    float pos_z_vel_split_power      = 0;
    float thrust_min                 = 0;
    float thrust_rate                = 0;
    float thrust_split_power_len     = 0;
    float thrust_xy_max              = 0;
    float thrust_xy_split_power_len  = 0;
    float vel_z                      = 0;
    float vel_z_error                = 0;
    float val_z_error_der            = 0;
    float delta_time                 = 0;
    float timestamp_now              = 0;

    static float timestamp_pre       = 0;
    static float vel_z_pre           = 0;

    // Get delta_time.
    // Ensure the calculation of delta_time isn't interrupt and keep update,
    // otherwise delta_time is too large and integration will saturate.
    if (timestamp_pre == 0)
    {
        timestamp_pre = Delay_GetRuntimeUs();
        return;
    }
    else
    {
        timestamp_now = Delay_GetRuntimeUs();
        delta_time    = (timestamp_now - timestamp_pre) / 1000000.0f;
        timestamp_pre = timestamp_now;
    }

    // Only in climb rate mode and landind mode, now we don't work on manual
    // mode.
    if ((control_altitude_mode == CONTROL_STATE_MANUAL) ||
        (!comm_link_fly_enable_flag))
    {
        return;
    }

    // Get current position of z.
    alt = -Altitude_NEDFrameStructure.pos_z;

    // Get desired rate of thrust.
    thrust_rate             = CommLink_DataStructure.thr / 1000.0f;
    // Scale to -1~1 in NED frame.
    split_power_z_move_rate = -CommLink_CutDBScaleToLinear(
        thrust_rate - 0.5f,
        0.5f,
        CONTROL_ALT_CTRL_Z_DB);
    // Scale to velocity max.
    split_power_z_move_rate = split_power_z_move_rate * CONTROL_ALT_VEL_MAX;

    // Get altitude's setpoint in Climb Rate mode.
    // The altitude is not in NED frame.
    alt_split_power   = -Altitude_NEDFrameStructure.pos_z;
    alt_split_power  -= split_power_z_move_rate * delta_time;

    // Limit altitude's setpoint.
    alt_split_power_offset_max = CONTROL_ALT_VEL_MAX / Control_PIDAlt.kp *
        2.0f;
    alt_split_power_offset     = alt_split_power - alt;

    if (alt_split_power_offset > alt_split_power_offset_max)
    {
        alt_split_power = alt + alt_split_power_offset_max;
    }
    else if (alt_split_power_offset < -alt_split_power_offset_max)
    {
        alt_split_power = alt - alt_split_power_offset_max;
    }

    // Limit height.
    if (altitude_limit_flag)
    {
        if (alt_split_power - control_alt_land > CONTROL_ALT_LIMIT)
        {
            alt_split_power = control_alt_land + CONTROL_ALT_LIMIT;
            split_power_z_move_rate = 0;
        }
    }

    // Set pid and feedforward control in NED frame.
    pos_z_error           = -(alt_split_power - alt);
    pos_z_vel_split_power = pos_z_error * Control_PIDAlt.kp +
        split_power_z_move_rate * CONTROL_ALT_FEED_FORWARD;

    // Consider landing mode.
    if (control_altitude_mode == CONTROL_STATE_LANDING)
    {
        pos_z_vel_split_power = CONTROL_LAND_SPEED;
    }

    // Get estimated value for hover.
    if (control_integral_reset_flag)
    {
        control_thrust_z_integral   = Control_EstimateThrustRefHover();
        control_integral_reset_flag = false;
    }

    vel_z           = Altitude_NEDFrameStructure.vel_z;
    vel_z_error     = pos_z_vel_split_power - vel_z;
    val_z_error_der = (split_power_z_move_rate - vel_z) * Control_PIDAlt.kp -
        (vel_z - vel_z_pre) / delta_time;
    vel_z_pre       = vel_z;
    // In NED frame, control_thrust_z_integral contains hover thrust.
    control_thrust_z_split_power = vel_z_error * Control_PIDAltVel.kp +
        val_z_error_der * Control_PIDAltVel.kd + control_thrust_z_integral;

    // Limit thrust's min.
    thrust_min = Control_EstimateThrustRefMin();

    if (control_altitude_mode != CONTROL_STATE_LANDING)
    {
        if (-control_thrust_z_split_power < thrust_min)
        {
            control_thrust_z_split_power = -thrust_min;
        }
    }

    // Relate to power split.
    saturation_xy_flag = false;
    saturation_z_flag  = false;
    // Convert angle to acceleration(target).
    thrust_xy_split_power[0]  = sinf(CommLink_DataStructure.roll * M_PI /
        180.0f);
    // Normalization.
    thrust_xy_split_power[1]  = sinf(CommLink_DataStructure.pitch * M_PI /
        180.0f);
    thrust_xy_split_power_len = sqrtf(thrust_xy_split_power[0] *
        thrust_xy_split_power[0] + thrust_xy_split_power[1] *
        thrust_xy_split_power[1]);

    // Limit tilt's max.
    if (thrust_xy_split_power_len > 0.01f)
    {
        thrust_xy_max = -control_thrust_z_split_power * tanf(CONTROL_TILT_MAX);
        if (thrust_xy_split_power_len > thrust_xy_max)
        {
            float k = thrust_xy_max / thrust_xy_split_power_len;
            thrust_xy_split_power[1] *= k;
            thrust_xy_split_power[0] *= k;
            saturation_xy_flag        = 1;
            thrust_xy_split_power_len = sqrtf(thrust_xy_split_power[0] *
                thrust_xy_split_power[0] + thrust_xy_split_power[1] *
                thrust_xy_split_power[1]);
        }
    }

    // Limit thrust's max.
    thrust_split_power_len = sqrtf(thrust_xy_split_power_len *
        thrust_xy_split_power_len + control_thrust_z_split_power *
        control_thrust_z_split_power);

    if (thrust_split_power_len > CONTROL_THRUST_MAX)
    {
        // Goint up.
        if (control_thrust_z_split_power < 0.0f)
        {
            if (-control_thrust_z_split_power > CONTROL_THRUST_MAX)
            {
                // Limit thrust z.
                thrust_xy_split_power[0]     = 0.0f;
                thrust_xy_split_power[1]     = 0.0f;
                control_thrust_z_split_power = -CONTROL_THRUST_MAX;
                saturation_xy_flag           = true;
                saturation_z_flag            = true;
            }
            else
            {
                // Preserve thrust z  and lower xy, keeping altitude is more
                // important than position.
                float k = 0;
                thrust_xy_max = sqrtf(CONTROL_THRUST_MAX * CONTROL_THRUST_MAX -
                    control_thrust_z_split_power *
                    control_thrust_z_split_power);
                k = thrust_xy_max / thrust_xy_split_power_len;
                thrust_xy_split_power[1] *= k;
                thrust_xy_split_power[0] *= k;
                saturation_xy_flag        = true;
            }
        }
        // Going down.
        else
        {   // Z component is negative, going down, simply limit thrust vector.
            float k = CONTROL_THRUST_MAX / thrust_split_power_len;
            control_thrust_z_split_power *= k;
            thrust_xy_split_power[1]     *= k;
            thrust_xy_split_power[0]     *= k;
            saturation_xy_flag            = true;
            saturation_z_flag             = true;
        }
    }

    split_power_roll  = asinf(thrust_xy_split_power[0]) * 180.0f / M_PI;
    split_power_pitch = asinf(thrust_xy_split_power[1]) * 180.0f / M_PI;

    // If appear saturation, don't integrate.
    if (!saturation_z_flag)
    {
        control_thrust_z_integral += vel_z_error * Control_PIDAltVel.ki * delta_time;
        if (control_thrust_z_integral > 0.0f)
        {
            control_thrust_z_integral = 0.0f;
        }
    }
}

void Control_SetHeadFreeMode(bool flag)
{
    if (flag)
    {
        head_yaw_angle      = IMU_TableStructure.yaw_ang;
        head_free_mode_flag = true;
    }
    else
    {
        head_free_mode_flag = false;
    }
}

void Control_SetMotorPWM(void)
{
    float tilt_cos = IMU_TableStructure.acc_b[2] / IMU_CONSTANTS_ONE_G;

    if (control_altitude_mode == CONTROL_STATE_MANUAL)
    {
        output_thrust = CommLink_DataStructure.thr;
        tilt_cos      = IMU_TableStructure.dcm_gb[2][2];
        output_thrust = output_thrust / tilt_cos;
    }
    else
    {
        // The tilt compensation effect is good, sometimes too fierce.
        output_thrust = (-control_thrust_z_split_power) * 1000;
        if (output_thrust > 1000)
        {
            output_thrust = 1000;
        }
    }

    // Fuse the values to motors.
    output_motor[0] = (s16)(output_thrust + output_pitch + output_roll -
        output_yaw);
    output_motor[1] = (s16)(output_thrust + output_pitch - output_roll +
        output_yaw);
    output_motor[2] = (s16)(output_thrust - output_pitch - output_roll -
        output_yaw);
    output_motor[3] = (s16)(output_thrust - output_pitch + output_roll +
        output_yaw);

    if (comm_link_fly_enable_flag)
    {
        Motor_SetPWM(output_motor[0], output_motor[1], output_motor[2],
                     output_motor[3]);
    }
    else
    {
        Motor_SetPWM(0, 0, 0, 0);
    }
}

// Estimate thrust's min value according to battery's voltage.
float Control_EstimateThrustRefMin(void)
{
    float min_thrust_ref = -0.55f;

    // Check battery voltage.
    Battery_InformationStructure.voltage_ad        = Battery_GetAD();
    Battery_InformationStructure.voltage_calculate =
        Battery_InformationStructure.voltage_factor *
       (Battery_InformationStructure.voltage_ad / 4096.0) *
        Battery_InformationStructure.voltage_ad_ref;

    if (Battery_InformationStructure.voltage_calculate > 4.05)
    {
        min_thrust_ref = -0.30f;
    }
    else if (Battery_InformationStructure.voltage_calculate > 3.90)
    {
        min_thrust_ref = -0.40f;
    }
    else
    {
        min_thrust_ref = -0.55f;
    }

    return min_thrust_ref;
}

// Estimate thrust's value for hover according to battery's voltage.
float Control_EstimateThrustRefHover(void)
{
    float hover_thrust_ref = -0.55f;

    // Check battery voltage.
    Battery_InformationStructure.voltage_ad        = Battery_GetAD();
    Battery_InformationStructure.voltage_calculate =
        Battery_InformationStructure.voltage_factor *
       (Battery_InformationStructure.voltage_ad / 4096.0) *
        Battery_InformationStructure.voltage_ad_ref;

    if (Battery_InformationStructure.voltage_calculate > 4.05)
    {
        hover_thrust_ref = -0.25f;
    }
    else if (Battery_InformationStructure.voltage_calculate > 3.90)
    {
        hover_thrust_ref = -0.40f;
    }
    else if (Battery_InformationStructure.voltage_calculate > 3.80)
    {
        hover_thrust_ref = -0.45f;
    }
    else if (Battery_InformationStructure.voltage_calculate > 3.70)
    {
        hover_thrust_ref = -0.50f;
    }
    else
    {
        hover_thrust_ref = -0.55f;
    }

    return hover_thrust_ref;
}
