/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_system_led.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Implement the led function
Others:      none
Function List:
             1. void LED_Init(void);
             2. void LED_JumpStateMachine(void);
             3. void LED_SetInitialLight(void);
             4. void LED_SetLight(LED_State led_a, LED_State led_b,
                                  LED_State led_c, LED_State led_d);
             5. void LED_UpdateLight(void);
History:
<author>    <date>        <desc>
maksyuki    2016.12.20    Modify the
myyerrol    2017.04.11    Format the module
*******************************************************************************/

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_algorithm_imu.h"

LED_Buffer       LED_BufferStructure;
LED_StateMachine LED_StateMachineStructure;

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // When hardware disables the JATG, don't disable the SWD meanwhile.
    // Otherwise the chip will be damaged!!!
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    LED_A_OFF;
    LED_B_OFF;
    LED_C_OFF;
    LED_D_OFF;
}

void LED_JumpStateMachine(void)
{
    LED_StateMachineStructure.state = LED_STATE_READY;

    if (!IMU_TableStructure.flag_ready)
    {
        // Start the calibration of IMU.
        LED_StateMachineStructure.state = LED_STATE_CALI;
    }
    if (!IMU_TableStructure.flag_cali)
    {
        // Fail to calibrate the IMU.
        LED_StateMachineStructure.state = LED_STATE_CALI_FAIL;
    }
    if (Battery_InformationStructure.flag_alarm)
    {
        LED_StateMachineStructure.state = LED_STATE_BAT_LOW;
    }
    if (imu_cali_flag)
    {
        // Finish the calibrate of IMU.
        LED_StateMachineStructure.state = LED_STATE_CALI;
    }
    if (Battery_InformationStructure.flag_charge)
    {
        LED_StateMachineStructure.state = LED_STATE_BAT_CHG;
    }

    switch (LED_StateMachineStructure.state)
    {
        case LED_STATE_READY:
        {
            if (++LED_StateMachineStructure.state_count >= 3)
            {
                LED_StateMachineStructure.state_count = 0;
            }
            if (LED_StateMachineStructure.state_count == 0)
            {
                LED_BufferStructure.byte = LED_A | LED_B;
            }
            else
            {
                LED_BufferStructure.byte = 0x00;
            }
            break;
        }
        case LED_STATE_CALI:
        {
            LED_BufferStructure.byte = LED_A | LED_B;
            break;
        }
        case LED_STATE_CALI_FAIL:
        {
            if (++LED_StateMachineStructure.state_count >= 4)
            {
                LED_StateMachineStructure.state_count = 0;
            }
            if (LED_StateMachineStructure.state_count < 2)
            {
                LED_BufferStructure.byte = LED_A | LED_B;
            }
            else
            {
                LED_BufferStructure.byte = LED_C | LED_D;
            }
            break;
        }
        case LED_STATE_BAT_LOW:
        {
            if (++LED_StateMachineStructure.state_count >= 3)
            {
                LED_StateMachineStructure.state_count = 0;
            }
            if (LED_StateMachineStructure.state_count == 0)
            {
                LED_BufferStructure.byte = 0x0F;
            }
            else
            {
                LED_BufferStructure.byte = 0x00;
            }
            break;
        }
        case LED_STATE_BAT_CHG:
        {
            LED_BufferStructure.byte = 0x00;
            break;
        }
    }

    LED_UpdateLight();
}

void LED_SetInitialLight(void)
{
    u8 i;

    for (i = 0; i < 4; i++)
    {
        LED_SetLight(ON, OFF, OFF, OFF);
        Delay_TimeMs(100);
        LED_SetLight(OFF, ON, OFF, OFF);
        Delay_TimeMs(100);
        LED_SetLight(OFF, OFF, ON, OFF);
        Delay_TimeMs(100);
        LED_SetLight(OFF, OFF, OFF, ON);
        Delay_TimeMs(100);
    }

    for (i = 0; i < 4; i++)
    {
        LED_SetLight(ON, ON, ON, ON);
        Delay_TimeMs(100);
        LED_SetLight(OFF, OFF, OFF, OFF);
        Delay_TimeMs(100);
    }
}

void LED_SetLight(LED_State led_a, LED_State led_b, LED_State led_c,
                  LED_State led_d)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, led_a);
    GPIO_WriteBit(GPIOA, GPIO_Pin_8,  led_b);
    GPIO_WriteBit(GPIOB, GPIO_Pin_1,  led_c);
    GPIO_WriteBit(GPIOB, GPIO_Pin_3,  led_d);
}

void LED_UpdateLight(void)
{
    if (LED_BufferStructure.bits.a)
    {
        LED_A_ON;
    }
    else
    {
        LED_A_OFF;
    }
    if (LED_BufferStructure.bits.b)
    {
        LED_B_ON;
    }
    else
    {
        LED_B_OFF;
    }
    if (LED_BufferStructure.bits.c)
    {
        LED_C_ON;
    }
    else
    {
        LED_C_OFF;
    }
    if (LED_BufferStructure.bits.d)
    {
        LED_D_ON;
    }
    else
    {
        LED_D_OFF;
    }
}
