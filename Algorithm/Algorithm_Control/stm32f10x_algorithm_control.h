#ifndef __STM32F10X_ALGORITHM_CONTROL_H__
#define __STM32F10X_ALGORITHM_CONTROL_H__

#include "stm32f10x.h"


#define  SLOW_THRO     200              /*怠速转速*/
#define  Angle_Max     40.0             /*定义飞机最大倾斜角度*/
#define  YAW_RATE_MAX  180.0f / M_PI_F  /*deg/s*/

//纠正姿态误差，可以用来抵抗重心偏移等带来的初始不平衡
//#define  Rool_error_init   7      //如果飞机起飞朝左偏，Rool_error_init朝正向增大修改;朝右偏，Rool_error_init朝负向增大修改
//#define  Pitch_error_init  -5      //如果飞机起飞朝前偏，Pitch_error_init朝负向增大修改;朝后偏，Pitch_error_init朝正向增大修改
//定高部分
#define LAND_SPEED   1.2f  /*m/s^2*/
#define ALT_VEL_MAX  4.0f

enum {CLIMB_RATE = 0, MANUAL, LANDING};
extern uint8_t altCtrlMode;
extern float   hoverThrust;
extern uint8_t zIntReset;
extern uint8_t offLandFlag;
extern float   altLand;
extern uint8_t isAltLimit;
extern float   thrustZSp;
extern float   thrustZInt;

// PID结构体
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


//写入Flash参数结构体
typedef struct
{
    u16 WriteBuf[10];  /*写入flash的临时数组*/
    u16 ReadBuf[10];   /*读取Flash的临时数组*/
}Parameter_Typedef;


extern void Controler(void);
extern void PID_INIT(void);
extern void PID_Calculate(void);

extern void CtrlAttiAng(void);
extern void CtrlAttiRate(void);
extern void CtrlAlti(void);
extern void CtrlAltiVel(void);
extern void CtrlMotor(void);
extern void CtrlTest(void);
extern void CtrlAttiRateNew(void);
extern void CtrlAttiNew(void);
extern float estimateHoverThru(void);

extern void SetHeadFree(uint8_t on);

extern u16 PIDWriteBuf[3];           /*写入flash的临时数字，由NRF24L01_RXDATA[i]赋值*/

extern PID_Typedef pitch_rate_PID;   /*pitch角速率环的PID*/
extern PID_Typedef pitch_angle_PID;  /*pitch角度环的PID*/

extern PID_Typedef yaw_rate_PID;     /*yaw的角速率环的PID*/
extern PID_Typedef yaw_angle_PID;    /*yaw角度环的PID*/

extern PID_Typedef roll_rate_PID;    /*roll角速率环的PID*/
extern PID_Typedef roll_angle_PID;   /*roll角度环的PID*/

extern PID_Typedef alt_PID;
extern PID_Typedef alt_vel_PID;


extern float gyroxGloble;
extern float gyroyGloble;

extern volatile unsigned char motorLock;

#endif
