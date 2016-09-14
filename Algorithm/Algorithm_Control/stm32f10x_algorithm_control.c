#include "stm32f10x_system_rpdata.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_it.h"
#include "math.h"


uint8_t offLandFlag = 0;

volatile unsigned char motorLock = 1;

int16_t Motor[4] = {0};   /*定义电机PWM数组，分别对应M1-M4*/
float rollSp  = 0;        /*根据动力分配重新计算得到的期望roll pitch*/
float pitchSp = 0;
float Thro    = 0;
float Pitch   = 0;
float Yaw     = 0;
float Roll    = 0;


//----PID结构体实例化----
PID_Typedef pitch_rate_PID;  /*pitch角速率环的PID*/
PID_Typedef pitch_angle_PID; /*pitch角度环的PID*/

PID_Typedef yaw_rate_PID;    /*yaw角速率环的PID*/
PID_Typedef yaw_angle_PID;   /*yaw角度环的PID*/

PID_Typedef roll_rate_PID;   /*roll角速率环的PID*/
PID_Typedef roll_angle_PID;  /*roll角度环的PID*/

PID_Typedef	alt_PID;
PID_Typedef alt_vel_PID;

float gyroxGloble = 0;
float gyroyGloble = 0;

//S_FLOAT_XYZ DIF_ACC;      /*实际去期望相差的加速度*/
//S_FLOAT_XYZ EXP_ANGLE;    /*期望角度*/
//S_FLOAT_XYZ DIF_ANGLE;    /*实际与期望相差的角度*/

float headHold       = 0;
uint32_t ctrlPrd     = 0;
uint8_t headFreeMode = 0;

/*函数名：PID_Postion_Cal()*/
/*描述：位置式PID*/
static void PID_Postion_Cal(PID_Typedef * PID, float target, float measure, int32_t dertT)
{
    float termI = 0;
    float dt    = dertT / 1000000.0;

    /*误差=期望值-测量值*/
    PID->Error    = target - measure;
    PID->Deriv    = (PID->Error - PID->PreError) / dt;
    PID->Output   = (PID->P * PID->Error) + (PID->I * PID->Integ) + (PID->D * PID->Deriv);  /*PID:比例环节+积分环节+微分环节*/
    PID->PreError = PID->Error;
    
    /*仅用于角度环和角速度环的*/
    if (FLY_ENABLE && offLandFlag)
    {
        if (fabs(PID->Output) < Thro)                  /*比油门还大时不积分*/
        {
            termI = (PID->Integ) + (PID->Error) * dt;  /*积分环节*/
            if ((termI > - PID->iLimit) && (termI < PID->iLimit) && (PID->Output > - PID->iLimit) 
                && (PID->Output < PID->iLimit))        /*在-300~300时才进行积分环节*/
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


/*函数名：CtrlAttiAng(void)*/
/*描述：对飞行器姿态控制（pitch，roll，yaw）控制中，串级PID中的角度环控制*/
void CtrlAttiAng(void)
{
    static uint32_t tPrev = 0;
    float angTarget[3]    = {0, 0, 0};
    float dt = 0, t = 0;
    t = micros();
    dt = (tPrev > 0) ? (t - tPrev) : 0;
    tPrev = t;

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

    PID_Postion_Cal(&pitch_angle_PID, angTarget[PITCH], imu.pitch, dt);
    PID_Postion_Cal(&roll_angle_PID, angTarget[ROLL], imu.roll, dt);
}


uint8_t altCtrlMode;					//normal=0  CLIMB rate, normal .  tobe tested