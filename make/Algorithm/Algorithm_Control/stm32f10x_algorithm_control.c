/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_algorithm_control.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.14
Description: implement the control function
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

#include "stm32f10x_driver_pwm.h"
#include "stm32f10x_module_rpdata.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_bar.h"
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_it.h"
#include "math.h"

uint8_t offLandFlag              = 0;
volatile unsigned char motorLock = 1;

float rollSp     = 0;    /* According to power distribution to calculate the expected roll pitch again */
float pitchSp    = 0;
float Yaw        = 0;
float Roll       = 0;
float Pitch      = 0;
float Thro       = 0;
int16_t Motor[4] = {0};  /* Correspond M1-M4 */

PID_Typedef pitch_rate_PID;
PID_Typedef pitch_angle_PID;

PID_Typedef yaw_rate_PID;
PID_Typedef yaw_angle_PID;

PID_Typedef roll_rate_PID;
PID_Typedef roll_angle_PID;

PID_Typedef	alt_PID;
PID_Typedef alt_vel_PID;

float gyroxGloble = 0;
float gyroyGloble = 0;

S_FLOAT_XYZ DIF_ACC;
S_FLOAT_XYZ EXP_ANGLE;
S_FLOAT_XYZ DIF_ANGLE;

float headHold       = 0;
uint32_t ctrlPrd     = 0;
uint8_t headFreeMode = 0;

/* Positional PID */
static void PID_PostionCal(PID_Typedef * PID, float target, float measure, int32_t dertT)
{
    float termI = 0;
    float dt = dertT / 1000000.0;

    PID->Error    = target - measure;
    PID->Deriv    = (PID->Error - PID->PreError) / dt;
    PID->Output   = (PID->P * PID->Error) + (PID->I * PID->Integ) + (PID->D * PID->Deriv);
    PID->PreError = PID->Error;

    /* Only use in angle and angular rate */
    if (FLY_ENABLE && offLandFlag)
    {
        if (fabs(PID->Output) < Thro)  /* Don't integrate when output is larger than thro */
        {
            termI = (PID->Integ) + (PID->Error) * dt;
            if ((termI > - PID->iLimit) && (termI < PID->iLimit) && (PID->Output > - PID->iLimit)
             && (PID->Output < PID->iLimit))  /* Integrate in -300~300 range */
            {
                PID->Integ = termI;
            }
        }
    }
    else
    {
        PID->Integ = 0;
    }
}

void SetHeadFree(uint8_t on)
{
    if (on == 1)
    {
        headHold = imu.yaw;
        headFreeMode = 1;
    }
    else
    {
        headFreeMode = 0;
    }
}

/* The angle cascade PID control */
void ControlAttiAng(void)
{
    float dt                = 0;
    float NowTime           = 0;
    float angTarget[3]      = {0, 0, 0};
    static uint32_t PreTime = 0;

    NowTime = micros();
    dt = (PreTime > 0) ? (NowTime - PreTime) : 0;
    PreTime = NowTime;

    if (altCtrlMode == MANUAL)
    {
        angTarget[ROLL]  = (float)NRF_Data.roll;
        angTarget[PITCH] = (float)NRF_Data.pitch;
    }
    else
    {
        angTarget[ROLL]  = rollSp;
        angTarget[PITCH] = pitchSp;
    }

    if (headFreeMode)
    {
#ifdef YAW_CORRECT
        float radDiff = -(imu.yaw - headHold) * M_PI_F / 180.0f;
#else
        float radDiff = (imu.yaw - headHold) * M_PI_F / 180.0f;
#endif
        float cosDiff    = cosf(radDiff);
        float sinDiff    = sinf(radDiff);
        float tarPitFree = angTarget[PITCH] * cosDiff + angTarget[ROLL] * sinDiff;
        angTarget[ROLL]  = angTarget[ROLL] * cosDiff - angTarget[PITCH] * sinDiff;
        angTarget[PITCH] = tarPitFree;
    }

    PID_PostionCal(&pitch_angle_PID, angTarget[PITCH], imu.pitch, dt);
    PID_PostionCal(&roll_angle_PID, angTarget[ROLL], imu.roll, dt);
}

/* The angular rate cascade PID control */
void ControlAttiRate(void)
{
    float dt                = 0;
    float NowTime           = 0;
    float yawRateTarget     = 0;
    static uint32_t PreTime = 0;

    NowTime = micros();
    dt = (PreTime > 0) ? (NowTime - PreTime) : 0;
    PreTime = NowTime;
    yawRateTarget = -(float)NRF_Data.yaw;

    /* Note: original PID parameters are in AD value, need to convert */
    PID_PostionCal(&pitch_rate_PID, pitch_angle_PID.Output, imu.gyro[PITCH] * 180.0f / M_PI_F, dt);
    PID_PostionCal(&roll_rate_PID, roll_angle_PID.Output, imu.gyro[ROLL] * 180.0f / M_PI_F, dt);  /* gyroxGloble */
    PID_PostionCal(&yaw_rate_PID, yawRateTarget, imu.gyro[YAW] * 180.0f / M_PI_F, dt);            /* DMP_DATA.GYROz */

    Pitch = pitch_rate_PID.Output;
    Roll  = roll_rate_PID.Output;
    Yaw   = yaw_rate_PID.Output;
}


#define ALT_FEED_FORWARD  0.5f
#define THR_MAX           1.0f  /* Max thrust */
#define ALT_LIMIT         2.0f  /* Limited height 3.5 */
#define TILT_MAX          (ANGLE_MAX * M_PI_F / 180.0)

const float ALT_CTRL_Z_DB = 0.1f;

float thrInit;
float altLand;
float spZMoveRate;

uint8_t altCtrlMode;           /* Normal = 0  CLIMB rate, normal. to be tested */
float hoverThrust   = 0;
uint8_t zIntReset   = 1;       /* Integral reset at first. when change manual mode to climb rate mode */
float thrustZInt    = 0;
float thrustZSp     = 0;
float thrustXYSp[2] = {0, 0};  /* Roll pitch */
uint8_t recAltFlag  = 0;
float holdAlt       = 0;
uint8_t satZ        = 0;       /* Whether the Z direction is saturated */
uint8_t satXY       = 0;       /* Whether Over-saturated */

uint8_t isAltLimit  = 0;

/* Get a estimated value for hold throttle. It will have a direct affection on hover */
/* Influence factor: battery voltage */
float EstimateHoverThru(void)
{
    float hoverHru = -0.55f;

    /* Check battery voltage */
    Battery_InitStructure.Battery_VoltageAD   = Battery_GetAD();
    Battery_InitStructure.Battery_VoltageCalculate = Battery_InitStructure.Battery_VoltageFactor * (Battery_InitStructure.Battery_VoltageAD / 4096.0) * Battery_InitStructure.Battery_VoltageAD_Ref;  /* Calculate actual voltage */

    if (Battery_InitStructure.Battery_VoltageCalculate > 4.05)
    {
        hoverHru = -0.25f;
    }
    else if (Battery_InitStructure.Battery_VoltageCalculate > 3.90)
    {
        hoverHru = -0.40f;
    }
    else if (Battery_InitStructure.Battery_VoltageCalculate > 3.80)
    {
        hoverHru = -0.45f;
    }
    else if (Battery_InitStructure.Battery_VoltageCalculate > 3.70)
    {
        hoverHru = -0.50f;
    }
    else
    {
        hoverHru = -0.55f;
    }

    return hoverHru;
}

/* Get a min estimated value for hold throttle depending on the weight and the battery level */
/* Throttle is so small that descent speed is too large, it will lead to unbalance. */
/* Influence factor: battery voltage */
float EstimateMinThru(void)
{
    float minThru = -0.55f;

    /* Check battery voltage */
    Battery_InitStructure.Battery_VoltageAD   = Battery_GetAD();
    Battery_InitStructure.Battery_VoltageCalculate = Battery_InitStructure.Battery_VoltageFactor * (Battery_InitStructure.Battery_VoltageAD / 4096.0) * Battery_InitStructure.Battery_VoltageAD_Ref;  /* Calculate actual voltage */

    if (Battery_InitStructure.Battery_VoltageCalculate > 4.05)
    {
        minThru = -0.30f;
    }
    else if (Battery_InitStructure.Battery_VoltageCalculate > 3.90)
    {
        minThru = -0.40f;
    }
    else
    {
        minThru = -0.55f;
    }

    return minThru;
}

/* Control height, the final result is in global variable 'thrustZSp' */
/* Only in climb rate mode and landind mode, now we don't work on manual mode */
void ControlAlti(void)
{
    float manThr          = 0;
    float alt             = 0;
    float velZ            = 0;
    float altSp           = 0;
    float posZVelSp       = 0;
    float altSpOffset;
    float altSpOffsetMax  = 0;
    float dt              = 0;
    float NowTime         = 0;
    static float PreTime  = 0;
    static float velZPrev = 0;
    float posZErr         = 0;
    float velZErr         = 0;
    float valZErrD        = 0;
    float thrustXYSpLen   = 0;
    float thrustSpLen     = 0;
    float thrustXYMax     = 0;
    float minThrust;

    /* Get dt */
    /* Ensure the calculation of dt isn't interrupt and keep update, */
    /* otherwise dt is too large and integration will saturate */
    if (PreTime == 0)
    {
        PreTime = micros();
        return;
    }
    else
    {
        NowTime = micros();
        dt = (NowTime - PreTime) / 1000000.0f;
        PreTime = NowTime;
    }

    /* Only in climb rate mode and landind mode, now we don't work on manual mode */
    if (MANUAL == altCtrlMode || !FLY_ENABLE)
    {
        return;
    }

    /* Pos z ctrol, get current alt */
    alt = -nav.z;

    /* Get desired move rate from stick */
    manThr      = NRF_Data.throttle / 1000.0f;
    spZMoveRate = -CutDBScaleToLinear(manThr - 0.5f, 0.5f, ALT_CTRL_Z_DB);  /* Scale to -1~1 . NED frame */
    spZMoveRate = spZMoveRate * ALT_VEL_MAX;                                /* Scale to vel min max */

    /* Get alt setpoint in CLIMB rate mode */
    altSp   = -nav.z;                                                       /* Only alt is not in ned frame. */
    altSp  -= spZMoveRate * dt;

    /* Limit alt setpoint */
    altSpOffsetMax = ALT_VEL_MAX / alt_PID.P * 2.0f;
    altSpOffset    = altSp - alt;

    if (altSpOffset > altSpOffsetMax)  /* Height increment restriction */
    {
        altSp = alt + altSpOffsetMax;
    }
    else if (altSpOffset < -altSpOffsetMax)
    {
        altSp = alt - altSpOffsetMax;
    }

    /* Limit height */
    if (isAltLimit)
    {
        if (altSp - altLand > ALT_LIMIT)
        {
            altSp = altLand + ALT_LIMIT;
            spZMoveRate = 0;
        }
    }

    /* PID and feedforward control. in ned frame */
    posZErr   = -(altSp - alt);
    posZVelSp = posZErr * alt_PID.P + spZMoveRate * ALT_FEED_FORWARD;

    /* Consider landing mode */
    if (altCtrlMode == LANDING)
    {
        posZVelSp = LAND_SPEED;
    }

    /* Get hold throttle. Give it a estimated value */
    if (zIntReset)
    {
        thrustZInt = EstimateHoverThru();
        zIntReset  = 0;
    }

    velZ      = nav.vz;
    velZErr   = posZVelSp - velZ;
    valZErrD  = (spZMoveRate - velZ) * alt_PID.P - (velZ - velZPrev) / dt;        /* spZMoveRate is from manual stick vel control */
    velZPrev  = velZ;
    thrustZSp = velZErr * alt_vel_PID.P + valZErrD * alt_vel_PID.D + thrustZInt;  /* In ned frame. thrustZInt contains hover thrust */

    /* Limit minimum down throttle */
    minThrust = EstimateMinThru();
    if (altCtrlMode != LANDING)
    {
        if (-thrustZSp < minThrust)
        {
            thrustZSp = -minThrust;
        }
    }

    /* Relate to Power distribution, testing */
    satXY = 0;
    satZ  = 0;
    thrustXYSp[0] = sinf(NRF_Data.roll * M_PI_F /180.0f);   /* Angle convert to acceleration(target) */
    thrustXYSp[1] = sinf(NRF_Data.pitch * M_PI_F /180.0f);  /* Normalize  */
    thrustXYSpLen = sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);

    /* Limit tilt max */
    if (thrustXYSpLen > 0.01f)
    {
        thrustXYMax = -thrustZSp * tanf(TILT_MAX);
        if (thrustXYSpLen > thrustXYMax)
        {
            float k = thrustXYMax / thrustXYSpLen;
            thrustXYSp[1] *= k;
            thrustXYSp[0] *= k;
            satXY          = 1;
            thrustXYSpLen  = sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
        }
    }

    /* Limit max thrust!! */
    thrustSpLen = sqrtf(thrustXYSpLen * thrustXYSpLen + thrustZSp * thrustZSp);
    if (thrustSpLen > THR_MAX)
    {
        if (thrustZSp < 0.0f)   /* Going up */
        {
            if (-thrustZSp > THR_MAX)
            {
                /* Thrust Z component is too large, limit it */
                thrustXYSp[0] = 0.0f;
                thrustXYSp[1] = 0.0f;
                thrustZSp     = -THR_MAX;
                satXY         = 1;
                satZ          = 1;
            }
            else
            {
                float k = 0;

                /* Preserve thrust Z component and lower XY, keeping altitude is more important than position */
                thrustXYMax = sqrtf(THR_MAX * THR_MAX - thrustZSp * thrustZSp);
                k = thrustXYMax / thrustXYSpLen;
                thrustXYSp[1] *= k;
                thrustXYSp[0] *= k;
                satXY = 1;
            }
        }
        else
        {       /* Going down */
                /* Z component is negative, going down, simply limit thrust vector */
                float k        = THR_MAX / thrustSpLen;
                thrustZSp     *= k;
                thrustXYSp[1] *= k;
                thrustXYSp[0] *= k;
                satXY          = 1;
                satZ           = 1;
        }
    }

    rollSp  = asinf(thrustXYSp[0]) * 180.0f / M_PI_F;
    pitchSp = asinf(thrustXYSp[1]) * 180.0f / M_PI_F;

    /* If saturation, don't integral */
    if (!satZ)  /* && fabs(thrustZSp)<THR_MAX */
    {
        thrustZInt += velZErr * alt_vel_PID.I * dt;
        if (thrustZInt > 0.0f)
        {
            thrustZInt = 0.0f;
        }
    }
}

void ControlMotor(void)
{
    float cosTilt = imu.accb[2] / CONSTANTS_ONE_G;
    if (altCtrlMode == MANUAL)
    {
        DIF_ACC.Z = imu.accb[2] - CONSTANTS_ONE_G;
        Thro      = NRF_Data.throttle;
        cosTilt   = imu.DCMgb[2][2];
        Thro      = Thro / cosTilt;
    }
    else
    {
        Thro = (-thrustZSp) * 1000;  /* imu.DCMgb[2][2]; The tilt compensation effect is good, sometimes too fierce */
        if (Thro > 1000)
        {
            Thro = 1000;
        }
    }

    /* Fuse the values to motors */
    Motor[2] = (int16_t)(Thro - Pitch - Roll - Yaw );  /* M3 */
    Motor[0] = (int16_t)(Thro + Pitch + Roll - Yaw );  /* M1 */
    Motor[3] = (int16_t)(Thro - Pitch + Roll + Yaw );  /* M4 */
    Motor[1] = (int16_t)(Thro + Pitch - Roll + Yaw );  /* M2 */

    if (FLY_ENABLE)
    {
        PWM_MotorFlash(Motor[0], Motor[1], Motor[2], Motor[3]);
    }
    else
    {
        PWM_MotorFlash(0, 0, 0, 0);
    }
}
