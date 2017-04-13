/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_system_led.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Declare the led function
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
maksyuki    2016.12.20    Modify the module
myyerrol    2017.04.11    Format the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_LED_H__
#define __STM32F10X_MODULE_LED_H__

#include "stm32f10x.h"

#define LED_A_ON     GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LED_A_OFF    GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define LED_B_ON     GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LED_B_OFF    GPIO_ResetBits(GPIOA, GPIO_Pin_8)

#define LED_C_ON     GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define LED_C_OFF    GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define LED_D_ON     GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LED_D_OFF    GPIO_ResetBits(GPIOB, GPIO_Pin_3)

#define LED_A_TOGGLE GPIO_WriteBit(GPIOA, GPIO_Pin_11, \
                                  !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_11))
#define LED_B_TOGGLE GPIO_WriteBit(GPIOA, GPIO_Pin_8,  \
                                  !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8))
#define LED_C_TOGGLE GPIO_WriteBit(GPIOB, GPIO_Pin_1,  \
                                  !GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1))
#define LED_D_TOGGLE GPIO_WriteBit(GPIOB, GPIO_Pin_3,  \
                                  !GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3))

#define LED_A 0x01
#define LED_B 0x02
#define LED_C 0x03
#define LED_D 0x04

#define LED_STATE_READY     0
#define LED_STATE_CALI      1
#define LED_STATE_CALI_FAIL 2
#define LED_STATE_BAT_LOW   3
#define LED_STATE_BAT_CHG   4
#define LED_STATE_LOST_RC   5

typedef enum
{
    OFF = 0,
    ON  = 1
} LED_State;

typedef union
{
    u8 byte;
    struct
    {
        u8 a : 1;
        u8 b : 1;
        u8 c : 1;
        u8 d : 1;
        u8   : 0;
    } bits;
} LED_Buffer;

typedef struct
{
    u8 state;
    u8 state_count;
} LED_StateMachine;

extern void LED_Init(void);
extern void LED_JumpStateMachine(void);
extern void LED_SetInitialLight(void);
extern void LED_SetLight(LED_State led_a, LED_State led_b, LED_State led_c,
                         LED_State led_d);
extern void LED_UpdateLight(void);

#endif
