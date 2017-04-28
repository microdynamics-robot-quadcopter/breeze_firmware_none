/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_comm_link.c
Author:      myyerrol
Version:     none
Create date: 2017.04.28
Description: Declare the communication operation function
Others:      none
Function List:
History:
<author>    <date>        <desc>
myyerrol    2017.04.28    Modify the module
*******************************************************************************/

#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_algorithm_imu.h"

u8   comm_link_mcu_state   = COMM_LINK_STATE_DISEN_MCU;
u16  comm_link_rc_data[4]  = {1500, 1500, 1500, 1500};
bool comm_link_fly_enable_flag = false;
CommLink_Data CommLink_DataStructure;

// Cut deadband scale to move linear.
float CommLink_CutDBScaleToLinear(float x_start, float x_end, float deadband)
{
    if (x_start > deadband)
    {
        return (x_start - deadband) / (x_end - deadband);
    }
    else if (x_start < -deadband)
    {
        return (x_start + deadband) / (x_end - deadband);
    }
    else
    {
        return 0.0F;
    }
}

void CommLink_ProcessDataFromNRF(void)
{
    if (altCtrlMode == LANDING)
    {
        comm_link_rc_data[ROLL]     = 1500;
        comm_link_rc_data[PITCH]    = 1500;
        comm_link_rc_data[YAW]      = 1500;
        comm_link_rc_data[THROTTLE] = 1500;
    }

    COMM_LINK_CONSTRAIN(comm_link_rc_data[ROLL], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[PITCH], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[YAW], 1000, 2000);
    COMM_LINK_CONSTRAIN(comm_link_rc_data[THROTTLE], 1000, 2000);

    CommLink_DataStructure.roll  = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[ROLL] - 1500), 500, COMM_LINK_APP_DB_PR) *
        ANGLE_MAX;
    CommLink_DataStructure.pitch = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[PITCH] - 1500), 500, COMM_LINK_APP_DB_PR) *
        ANGLE_MAX;
    CommLink_DataStructure.yaw   = CommLink_CutDBScaleToLinear(
        (comm_link_rc_data[YAW] - 1500), 500, COMM_LINK_APP_DB_YAW) *
        YAW_RATE_MAX;
    CommLink_DataStructure.throttle = comm_link_rc_data[THROTTLE] - 1000;

    switch (comm_link_mcu_state)
    {
        case COMM_LINK_STATE_REQ_EN_MCU:
        {
            if (IMU_Check() && !Battery_InformationStructure.flag_alarm)
            {
                comm_link_mcu_state       = COMM_LINK_STATE_EN_MCU;
                comm_link_fly_enable_flag = true;
                LED_C_ON;
            }
            else
            {
                comm_link_mcu_state       = COMM_LINK_STATE_DISEN_MCU;
                comm_link_fly_enable_flag = false;
            }
            break;
        }
        case COMM_LINK_STATE_REQ_DISEN_MCU:
        {
            comm_link_mcu_state        = COMM_LINK_STATE_DISEN_MCU;
            comm_link_fly_enable_flag  = false;
            altCtrlMode                = MANUAL;
            zIntReset                  = 1;
            thrustZSp                  = 0;
            thrustZInt                 = EstimateHoverThru();
            offLandFlag                = 0;
            break;
        }
        default:
        {
            break;
        }
    }
}

void CommLink_ReceiveDataFromNRF(void)
{
    if ((nrf24l01_rx_data[0] == '$') && (nrf24l01_rx_data[1] == 'M') &&
        (nrf24l01_rx_data[2] == '<'))
    {
        switch (nrf24l01_rx_data[4])
        {
            case COMM_LINK_MSP_SET_4CON:
            {
                comm_link_rc_data[ROLL]     = nrf24l01_rx_data[11] +
                    (nrf24l01_rx_data[12] << 8);
                comm_link_rc_data[PITCH]    = nrf24l01_rx_data[9]  +
                    (nrf24l01_rx_data[10] << 8);
                comm_link_rc_data[YAW]      = nrf24l01_rx_data[7]  +
                    (nrf24l01_rx_data[8]  << 8);
                comm_link_rc_data[THROTTLE] = nrf24l01_rx_data[5]  +
                    (nrf24l01_rx_data[6]  << 8);
                break;
            }
            case COMM_LINK_MSP_ARM_IT:
            {
                comm_link_mcu_state = COMM_LINK_STATE_REQ_EN_MCU;
                break;
            }
            case COMM_LINK_MSP_DISARM_IT:
            {
                comm_link_mcu_state = COMM_LINK_STATE_REQ_DISEN_MCU;
                break;
            }
            case COMM_LINK_MSP_ACC_CALI:
            {
                imuCaliFlag = 1;
                break;
            }
        }
    }
}
