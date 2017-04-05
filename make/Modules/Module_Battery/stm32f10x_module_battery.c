/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_battery.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: implement the battery operation function
Others:      none
Function List:
             1. int  Battery_GetAD(void);
             2. int  Battery_GetTemp(void);
             3. void Battery_Check(void);
             4. void Battery_CheckInit(void);
             5. u16  Battery_GetADC(u8 ch);
             6. u16  Battery_GetADCAverage(u8 ch, u8 times);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.30  modify the module
*******************************************************************************/

#include "stm32f10x_module_rpdata.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_algorithm_control.h"
#include "stdio.h"

extern uint8_t FLY_ENABLE;

Bat_TypedefStructure Battery;

/* Get the ADC value from internal temperature sensor */
/* Return three bits temperature value XXX * 0.1c */
int Battery_GetTemp(void)
{
    u16 temp_val = 0;
    float temperature;

    u8 i;
    for (i = 0; i < 20; i++)
    {
        temp_val += Battery_GetADC(16); /* 16 channels */
    }

    temp_val    /= 20;
    temperature  = (float)temp_val * (3.3 / 4096);     /* Get the voltage value */
    temperature  = (1.43 - temperature) / 0.0043 + 25; /* Calculate the temperature */
    temperature *= 10;
    return (int)temperature;
}

int Battery_GetAD(void)
{
    return Battery_GetADCAverage(8, 5);
}

void Battery_Check(void)
{
    Battery.ADVal   = Battery_GetAD(); /* Detect battery voltage */
    Battery.RealVal = Battery.Factor * (Battery.ADVal / 4096.0) * Battery.ADRefVal;

    if (FLY_ENABLE)
    {
        /* Under the state of idling */
        if (Battery.RealVal <= (BAT_OVERDIS_VAL + 0.03))
        {
            Battery.AlarmFlag = 1;
        }
        else
        {
            Battery.AlarmFlag = 0;
        }

        /* Battery overdischarge protect */
        if (Battery.RealVal <= BAT_OVERDIS_VAL)
        {
            Battery.OverDischargeCnt++;
            if (Battery.OverDischargeCnt > 8)
            {
                altCtrlMode = LANDING;
                rcData[0]   = 1500;
                rcData[1]   = 1500;
                rcData[2]   = 1500;
                rcData[3]   = 1500;
            }
        }
        else
        {
            Battery.OverDischargeCnt = 0;
        }
    }
    else
    {
        if ((Battery.RealVal < BAT_ALARM_VAL) && (Battery.RealVal > BAT_CHARGE_VAL))
        {
            Battery.AlarmFlag = 1;
        }
        else
        {
            Battery.AlarmFlag = 0;
        }
    }

    /* On charge */
    if (Battery.RealVal < BAT_CHARGE_VAL)
    {
        Battery.ChargeState = 1;
    }
    else
    {
        Battery.ChargeState = 0;
    }
}

/* Start the channel 8 of ADC1 */
/* BatteryCheck---->PB0 */
void Battery_CheckInit(void)
{
    /* Initialize the PB0 and set it become analog input mode */
    RCC->APB2ENR  |= 1<<3;        /* Enable the clock of PORTB */
    GPIOB->CRL    &= 0XFFFFFFF0;  /* PB0 anolog input */

    /* Channel 8 */
    RCC->APB2ENR  |= 1<<9;        /* Enable the clock of ADC1 */
    RCC->APB2RSTR |= 1<<9;        /* Reset the ADC1 */
    RCC->APB2RSTR &= ~(1<<9);     /* Reset is end */
    RCC->CFGR     &= ~(3<<14);    /* Clear frequency division factor */

    /* SYSCLK/DIV2=12M set the clock of ADC is 12M, the maximum value cannot excess 14M! */
    /* Otherwise it can lead to the low accuracy in getting ADC! */
    RCC->CFGR  |= 2<<14;
    ADC1->CR1  &= 0XF0FFFF;       /* Clear the working mode */
    ADC1->CR1  |= 0<<16;          /* Stand-alone mode */
    ADC1->CR1  &= ~(1<<8);        /* Non scanning mode */
    ADC1->CR2  &= ~(1<<1);        /* Single conversion mode */
    ADC1->CR2  &= ~(7<<17);
    ADC1->CR2  |= 7<<17;          /* Control conversion by software */
    ADC1->CR2  |= 1<<20;          /* Use external trigger(SWSTART)!!! must use an event to trigger */
    ADC1->CR2  &= ~(1<<11);       /* Right align */
    ADC1->CR2  |= 1<<23;          /* Enalbe the temperature sensor */
    ADC1->SQR1 &= ~(0XF<<20);
    ADC1->SQR1 &= 0<<20;          /* One conversion is in the regular sequence */

    /* Set the sample time of channel 1 */
    ADC1->SMPR2 &= ~(7<<3);       /* Clear the sample time of channel 1 */
    ADC1->SMPR2 |= 7<<3;          /* The sample time is(channel 1) 239.5 period, improving it can improve accuracy */

    ADC1->SMPR1 &= ~(7<<18);      /* Clear original settings of channel 16 */
    ADC1->SMPR1 |= 7<<18;         /* The sample time is(channel 16) 239.5 period, improving it can improve accuracy */

    ADC1->CR2 |= 1<<0;            /* Start the AD converter */
    ADC1->CR2 |= 1<<3;            /* Enable the reset calibration */
    while (ADC1->CR2 & (1<<3));   /* Wait the end of calibration, this bit is set by software and cleared by hardware */
                                  /* After the initialization of calibration register, this bit will be cleared */

    ADC1->CR2 |= 1<<2;            /* Start the AD calibration */
    while (ADC1->CR2 & (1<<2));   /* Wait the end of calibration, this bit is set by software and cleared by hardware */

    /* Unit: v, they are modified when calibrating voltage */
    Battery.TestVal          = 3.95;
    Battery.ADInputVal       = 1.98;
    Battery.ADRefVal         = 3.26;
    Battery.Factor           = Battery.TestVal / Battery.ADInputVal;
    Battery.OverDischargeCnt = 0;

    printf("Batter voltage AD init ...\r\n");
}

/* ch: 0~16 channel */
/* Return: conversion result */
u16 Battery_GetADC(u8 ch)
{
    /* Set conversion sequence */
    ADC1->SQR3 &= 0XFFFFFFE0;     /* The channel of regular sequence 1 */
    ADC1->SQR3 |= ch;
    ADC1->CR2  |= 1<<22;          /* Start the regular sequence conversion channel */
    while (!(ADC1->SR & (1<<1))); /* Wait the end of conversion */
    return ADC1->DR;              /* Return ADC value */
}

/* ch: channel id */
/* times: the numbers of getting ADC */
/* Return: the average value of channel ch */
u16 Battery_GetADCAverage(u8 ch, u8 times)
{
    u32 temp_val = 0;

    u8 i;
    for (i = 0; i < times; i++)
    {
        temp_val += Battery_GetADC(ch);
    }
    return temp_val / times;
}
