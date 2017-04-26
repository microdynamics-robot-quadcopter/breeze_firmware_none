/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_rpdata.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.20
Description: implement the data receive and process operation function
Others:      none
Function List:
             1. void ReceiveDataFromNRF(void);
             2. void ProcessDataFromNRF(void);
             3. float CutDBScaleToLinear(float x, float x_end,
                                         float deadband);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.22  modify the module
*******************************************************************************/

#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_led.h"  /* Debug */
#include "stm32f10x_module_rpdata.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_motor.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_control.h"
#include "stdio.h"

NRF_GetData NRF_Data;
uint8_t  FLY_ENABLE = 0;
uint8_t  armState   = DISARMED;
uint16_t rcData[4]  = {1500, 1500, 1500, 1500};

void ReceiveDataFromNRF(void)
{
    if ((nrf24l01_rx_data[0] == '$') && (nrf24l01_rx_data[1] == 'M') && (nrf24l01_rx_data[2] == '<'))
    {
        switch (nrf24l01_rx_data[4])
        {
            case MSP_SET_4CON:
                rcData[THROTTLE] = nrf24l01_rx_data[5]  + (nrf24l01_rx_data[6]  << 8);
                rcData[YAW]      = nrf24l01_rx_data[7]  + (nrf24l01_rx_data[8]  << 8);
                rcData[PITCH]    = nrf24l01_rx_data[9]  + (nrf24l01_rx_data[10] << 8);
                rcData[ROLL]     = nrf24l01_rx_data[11] + (nrf24l01_rx_data[12] << 8);
            break;

            case MSP_ARM_IT:
                armState = REQ_ARM;
            break;

            case MSP_DISARM_IT:
                armState = REQ_DISARM;
            break;

            case MSP_ACC_CALI:
                imuCaliFlag = 1;
            break;
        }
    }
}

void ProcessDataFromNRF(void)
{
    if (LANDING == altCtrlMode)
    {
        rcData[THROTTLE] = 1500;
        rcData[YAW]      = 1500;
        rcData[PITCH]    = 1500;
        rcData[ROLL]     = 1500;
    }
    CONSTRAIN(rcData[PITCH], 1000, 2000);
    CONSTRAIN(rcData[YAW], 1000, 2000);
    CONSTRAIN(rcData[ROLL], 1000, 2000);
    CONSTRAIN(rcData[THROTTLE], 1000, 2000);

    NRF_Data.throttle = rcData[THROTTLE] - 1000;
    NRF_Data.yaw   = YAW_RATE_MAX * CutDBScaleToLinear((rcData[YAW] - 1500), 500, APP_YAW_DB);
    NRF_Data.pitch = ANGLE_MAX    * CutDBScaleToLinear((rcData[PITCH] - 1500), 500, APP_PR_DB);
    NRF_Data.roll  = ANGLE_MAX    * CutDBScaleToLinear((rcData[ROLL] - 1500), 500, APP_PR_DB);

    // printf("This is the value of the armState:\n");  /* Debug */
    // printf("%d\n", armState);

//    if (armState == REQ_ARM)
//    {
//        Motor_SetPWM(200, 200, 200, 200);
//    }
//
//    if (armState == REQ_DISARM)
//    {
//        Motor_SetPWM(0, 0, 0, 0);
//    }
    switch (armState)
    {
        case REQ_ARM:
            if (IMU_Check() && !Battery_InformationStructure.flag_alarm)
            {
                armState = ARMED;
                FLY_ENABLE = 0xA5;
                LED_C_ON;

                /* This is an unuseful process, because it will be assigned zero in 100Hz loop */
                /* Setting in the flight module */
                //Motor_SetPWM(200, 200, 200, 200);
            }
            else
            {
                armState = DISARMED;
                FLY_ENABLE = 0;
            }
        break;

        case REQ_DISARM:
            armState    = DISARMED;
            FLY_ENABLE  = 0;
            altCtrlMode = MANUAL;       /* Extra process after DISARMED */
            zIntReset   = 1;
            thrustZSp   = 0;
            thrustZInt  = EstimateHoverThru();
            offLandFlag = 0;
        break;

        default:
            break;
    }
}

/* Cut deadband scale to move linear */
float CutDBScaleToLinear(float x, float x_end, float deadband)
{
    if (x > deadband)
    {
        return (x - deadband) / (x_end - deadband);
    }
    else if (x < -deadband)
    {
        return (x + deadband) / (x_end - deadband);
    }
    else
    {
        return 0.0f;
    }
}
