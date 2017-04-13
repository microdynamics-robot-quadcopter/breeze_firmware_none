/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_battery.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Implement the battery operation function
Others:      none
Function List:
             1. void Battery_Check(void);
             2. void Battery_Init(void);
             3. u16  Battery_GetADC(u8 ch);
             4. u16  Battery_GetADCAverage(u8 ch, u8 times);
             5. s32  Battery_GetAD(void);
             6. s32  Battery_GetTemp(void);
History:
<author>    <date>        <desc>
maksyuki    2016.12.30    Modify the module
myyerrol    2017.04.11    Format the module
*******************************************************************************/

#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_rpdata.h"
#include "stm32f10x_algorithm_control.h"

extern uint8_t FLY_ENABLE;

Battery_Information Battery_InformationStructure;

void Battery_Check(void)
{
    // Detect the voltage of battery.
    Battery_InformationStructure.voltage_ad = Battery_GetAD();
    // Calculate the real voltage of battery.
    Battery_InformationStructure.voltage_calculate =
        Battery_InformationStructure.voltage_factor
     * (Battery_InformationStructure.voltage_ad / 4096.0)
     *  Battery_InformationStructure.voltage_ad_ref;

    if (FLY_ENABLE)
    {
        if (Battery_InformationStructure.voltage_calculate
        <= (BATTERY_VOLTAGE_OVERDIS + 0.03))
        {
            Battery_InformationStructure.flag_alarm = TRUE;
        }
        else
        {
            Battery_InformationStructure.flag_alarm = FALSE;
        }

        if (Battery_InformationStructure.voltage_calculate
        <=  BATTERY_VOLTAGE_OVERDIS)
        {
            Battery_InformationStructure.over_discharge_cnt++;
            if (Battery_InformationStructure.over_discharge_cnt > 8)
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
            Battery_InformationStructure.over_discharge_cnt = 0;
        }
    }
    else
    {
        if ((Battery_InformationStructure.voltage_calculate
        < BATTERY_VOLTAGE_ALARM)
        && (Battery_InformationStructure.voltage_calculate
        > BATTERY_VOLTAGE_CHARGE))
        {
            Battery_InformationStructure.flag_alarm = TRUE;
        }
        else
        {
            Battery_InformationStructure.flag_alarm = FALSE;
        }
    }

    if (Battery_InformationStructure.voltage_calculate < BATTERY_VOLTAGE_CHARGE)
    {
        Battery_InformationStructure.flag_charge = TRUE;
    }
    else
    {
        Battery_InformationStructure.flag_charge = FALSE;
    }
}

void Battery_Init(void)
{
    // ADC_InitTypeDef  ADC_InitStructure;
    // GPIO_InitTypeDef GPIO_InitStructure;
    //
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE);
    // RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    //
    // GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);
    //
    // ADC_DeInit(ADC1);
    // ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    // ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
    // ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    // ADC_InitStructure.ExternalTrigConv       = ADC_ExternalTrigConv_None;
    // ADC_InitStructure.DataAlign              = ADC_DataAlign_Right;
    // ADC_InitStructure.NbrOfChannel           = 1;
    //
    // ADC_Cmd(ADC1, ENABLE);
    //
    // ADC_ResetCalibration(ADC1);
    // while (ADC_GetResetCalibrationStatus(ADC1));
    //
    // ADC_StartCalibration(ADC1);
    // while (ADC_GetCalibrationStatus(ADC1));

    // Initialize the PB0 and set it become analog input mode.
    // Enable the clock of PORTB.
    RCC->APB2ENR  |= 1 << 3;
    // PB0 anolog input.
    GPIOB->CRL    &= 0XFFFFFFF0;

    // Channel 8.
    // Enable the clock of ADC1.
    RCC->APB2ENR  |= 1 << 9;
    // Reset the ADC1.
    RCC->APB2RSTR |= 1 << 9;
    // Reset is end.
    RCC->APB2RSTR &= ~(1 << 9);
    // Clear frequency division factor.
    RCC->CFGR     &= ~(3 << 14);

    // SYSCLK/DIV2=12M set the clock of ADC is 12M, the maximum value cannot
    // excess 14M! Otherwise it can lead to the low accuracy in getting ADC!
    RCC->CFGR  |= 2 << 14;
    // Clear the working mode.
    ADC1->CR1  &= 0XF0FFFF;
    // Stand-alone mode.
    ADC1->CR1  |= 0 << 16;
    // Non scanning mode.
    ADC1->CR1  &= ~(1 << 8);
    // Single conversion mode.
    ADC1->CR2  &= ~(1 << 1);
    ADC1->CR2  &= ~(7 << 17);
    // Control conversion by software.
    ADC1->CR2  |= 7 << 17;
    // Use external trigger(SWSTART)!!! Use an event to trigger.
    ADC1->CR2  |= 1 << 20;
    // Right align.
    ADC1->CR2  &= ~(1 << 11);
    // Enalbe the temperature sensor.
    ADC1->CR2  |= 1 << 23;

    ADC1->SQR1 &= ~(0XF << 20);
    // One conversion is in the regular sequence.
    ADC1->SQR1 &= 0 << 20;

    // Set the sample time of channel 1.
    // Clear the sample time of channel 1.
    ADC1->SMPR2 &= ~(7 << 3);
    // The sample time is(channel 1) 239.5 period, increasing it can improve
    // accuracy.
    ADC1->SMPR2 |= 7 << 3;

    // Clear original settings of channel 16.
    ADC1->SMPR1 &= ~(7 << 18);
    // The sample time is(channel 16) 239.5 period, increasing it can improve
    // accuracy.
    ADC1->SMPR1 |= 7 << 18;

    // Start the AD converter.
    ADC1->CR2 |= 1 << 0;
    // Enable the reset calibration.
    ADC1->CR2 |= 1 << 3;
    // Wait the end of calibration, this bit is set by software and cleared by
    // hardware. After the initialization of calibration register, this bit
    // will be cleared.
    while (ADC1->CR2 & (1 << 3));
    // Start the AD calibration.
    ADC1->CR2 |= 1 << 2;
    // Wait the end of calibration, this bit is set by software and cleared by
    // hardware.
    while (ADC1->CR2 & (1 << 2));

    Battery_InformationStructure.voltage_measure    = 3.95;
    Battery_InformationStructure.voltage_ad_in      = 1.98;
    Battery_InformationStructure.voltage_ad_ref     = 3.26;
    Battery_InformationStructure.voltage_factor     =
        Battery_InformationStructure.voltage_measure
      / Battery_InformationStructure.voltage_ad_in;
    Battery_InformationStructure.over_discharge_cnt = 0;

    // printf("Init the module of battery successfully!\r\n");
}

u16 Battery_GetADC(u8 ch)
{
    // ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
    // ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //
    // while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    //
    // return ADC_GetConversionValue(ADC1);

    // Set conversion sequence.
    // The channel of regular sequence 1.
    ADC1->SQR3 &= 0XFFFFFFE0;
    ADC1->SQR3 |= ch;
    // Start the regular sequence conversion channel.
    ADC1->CR2  |= 1 << 22;
    // Wait the end of conversion.
    while (!(ADC1->SR & (1 << 1)));
    // Return the value of ADC.
    return ADC1->DR;
}

u16 Battery_GetADCAverage(u8 ch, u8 times)
{
    u8 i;
    u32 temp = 0;

    for (i = 0; i < times; i++)
    {
        temp += Battery_GetADC(ch);
    }

    return temp / times;
}

s32 Battery_GetAD(void)
{
    return Battery_GetADCAverage(8, 5);
}

s32 Battery_GetTemp(void)
{
    u8 i;
    u16 temp = 0;
    float temperature;

    for (i = 0; i < 20; i++)
    {
        temp += Battery_GetADC(16);
    }

    temp        /= 20;
    // Get the voltage value.
    temperature  = (float)temp * (3.3 / 4096);
    // Calculate the temperature.
    temperature  = (1.43 - temperature) / 0.0043 + 25;
    temperature *= 10;

    return (int)temperature;
}
