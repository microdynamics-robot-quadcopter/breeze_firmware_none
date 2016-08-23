#include "stm32f10x_system_control.h"

uint8_t offLandFlag=0;

volatile unsigned char motorLock = 1;

int16_t Motor[4] = {0};   //定义电机PWM数组，分别对应M1-M4
float rollSp = 0, pitchSp = 0;		//根据动力分配重新计算得到的期望roll pitch
float Thro = 0, Roll = 0, Pitch = 0, Yaw = 0;


//----PID结构体实例化----
PID_Typedef pitch_angle_PID;	//pitch角度环的PID
PID_Typedef pitch_rate_PID;		//pitch角速率环的PID

PID_Typedef roll_angle_PID;   //roll角度环的PID
PID_Typedef roll_rate_PID;    //roll角速率环的PID

PID_Typedef yaw_angle_PID;    //yaw角度环的PID 
PID_Typedef yaw_rate_PID;     //yaw角速率环的PID

PID_Typedef	alt_PID;
PID_Typedef alt_vel_PID;

float gyroxGloble = 0;
float gyroyGloble = 0;


//S_FLOAT_XYZ DIF_ACC;		//实际去期望相差的加速度
//S_FLOAT_XYZ EXP_ANGLE;	//期望角度	
//S_FLOAT_XYZ DIF_ANGLE;	//实际与期望相差的角度	

uint32_t ctrlPrd=0;
uint8_t headFreeMode=0;
float headHold=0;
