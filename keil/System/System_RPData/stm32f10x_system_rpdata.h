#ifndef __STM32F10X_SYSTEM_RPDATA_H__
#define __STM32F10X_SYSTEM_RPDATA_H__

#include "stm32f10x.h"

enum {DISARMED = 0, REQ_ARM, ARMED, REQ_DISARM};
//enum {ROLL, PITCH, YAW, THROTTLE};

#define MSP_SET_THRO        1
#define MSP_SET_YAW         2
#define MSP_SET_PITCH       3
#define MSP_SET_ROLL        4
#define MSP_ARM_IT          5
#define MSP_DISARM_IT       6
#define MSP_SET_4CON        7
#define MSP_SETOFF          8
#define MSP_LAND_DOWN       9
#define MSP_HOLD_ALT        10
#define MSP_STOP_HOLD_ALT   11
#define MSP_HEAD_FREE       12
#define MSP_STOP_HEAD_FREE  13
#define MSP_POS_HOLD        14
#define MSP_STOP_POS_HOLD   15
#define MSP_FLY_STATE       16
#define MSP_ACC_CALI        205

#define APP_YAW_DB  70  /*dead band*/
#define APP_PR_DB   50

#define  M_PI_F        3.1415926
#define  ANGLE_MAX     40.0             /*定义飞机最大倾斜角度*/
#define  YAW_RATE_MAX  180.0f / M_PI_F  /*deg/s*/


#define CONSTRAIN(x, min, max) {if (x < min) x = min; if (x > max) x = max;}

typedef struct NRF
{
    float pitch;
    float yaw;    
    float roll;
    float throttle;
}NRF_GetData;

extern NRF_GetData NRF_Data;
extern uint8_t FLY_ENABLE;

extern uint8_t  appCmdFlag;
extern uint8_t  flyLogApp;
extern uint8_t  armState;
extern uint16_t rcData[4];

extern void CommApp(u8 ch);
extern void CommAppUpload(void);
extern void ReceiveDataFromNRF(void);
extern void ProcessDataFromNRF(void);
extern float dbScaleLinear(float x, float x_end, float deadband);
#endif
