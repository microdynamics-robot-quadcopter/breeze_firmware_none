/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_system_battery.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.14
Description: implement the battery operation function
Others:      none
Function List:
             1. extern int Get_Temp(void);
             2. extern int Get_Battery_AD(void);
             3. extern void Battery_Check(void);
             4. extern void Battery_Check_Init(void);
             5. extern u16 Get_ADC(u8 ch);
             6. extern u16 Get_ADC_Average(u8 ch, u8 times);
History:     none
*******************************************************************************/

#include "stm32f10x_system_battery.h"
#include "stdio.h"

extern uint8_t FLY_ENABLE;

/*实例化一个电压信息结构体*/
Bat_TypedefStructure Battery;

/*得到ADC采样内部温度传感器的温度值*/
/*返回值3位温度值 XXX*0.1c*/
int Get_Temp(void)
{
    u16 temp_val = 0;
    float temperate;

    u8 i;
    for (i = 0; i < 20; i++)                       /*读20次，取平均值*/
    {
        temp_val += Get_ADC(16);                   /*温度传感器为通道16*/
    }
    
    temp_val /= 20;
    temperate = (float)temp_val * (3.3 / 4096);    /*得到温度传感器的电压值*/
    temperate = (1.43 - temperate) / 0.0043 + 25;  /*计算出当前温度值*/
    temperate *= 10;                               /*扩大十倍,使用小数点后一位*/
    return (int)temperate;
}

/*返回电池电压AD值*/
int Get_Battery_AD(void)
{
    return Get_ADC_Average(8, 5);
}

/*检测电池电压*/
void Battery_Check(void)
{
    Battery.ADVal  = Get_Battery_AD();                                               /*电池电压检测*/
    Battery.RealVal = Battery.Factor * (Battery.ADVal / 4096.0) * Battery.ADRefVal;  /*实际电压 值计算*/

    if (FLY_ENABLE)
    {
        /*处于电机开启等飞行状态，在过放电压值（BAT_OVERDIS_VAL）以上0.03v以上，开始报警*/
        if (Battery.RealVal <= (BAT_OVERDIS_VAL + 0.03))
        {
            Battery.AlarmFlag = 1;
        }
        else
        {
            Battery.AlarmFlag = 0;
        }

        /*过放保护，Battery overdischarge protect*/
        if (Battery.RealVal <= BAT_OVERDIS_VAL)
        {
            Battery.OverDischargeCnt++;
            if (Battery.OverDischargeCnt > 8)
            {
                //altCtrlMode=LANDING;
                //rcData[0]=1500;rcData[1]=1500;rcData[2]=1500;rcData[3]=1500;
            }
        }
        else
        {
            Battery.OverDischargeCnt = 0;
        }
	}
    else
    {
        if ((Battery.RealVal < BAT_ALARM_VAL) && (Battery.RealVal > BAT_CHARGE_VAL))  /*低于3.7v 且大于充电检测电压 BAT_CHG_VAL*/
        {
            Battery.AlarmFlag = 1;
        }
        else
        {
            Battery.AlarmFlag = 0;
        }
    }

    if (Battery.RealVal < BAT_CHARGE_VAL)  /*on charge*/
    {
        Battery.ChargeState = 1;
    }
    else
    {
        Battery.ChargeState = 0;
    }
}

/*初始化电池检测ADC*/
/*开启ADC1的通道8*/
/*BatteryCheck---->PB0*/
void Battery_Check_Init(void)
{
    /*先初PB0为模拟输入*/
    RCC->APB2ENR  |= 1<<3;        /*使能PORTB口时钟*/
    GPIOB->CRL    &= 0XFFFFFFF0;  /*PB0	anolog输入*/

    /*通道8*/
    RCC->APB2ENR  |= 1<<9;        /*ADC1时钟使能*/
    RCC->APB2RSTR |= 1<<9;        /*ADC1复位*/
    RCC->APB2RSTR &= ~(1<<9);     /*复位结束*/
    RCC->CFGR     &= ~(3<<14);    /*分频因子清零*/

    /*SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!*/
    /*否则将导致ADC准确度下降!*/
    RCC->CFGR |= 2<<14;
    ADC1->CR1 &= 0XF0FFFF;        /*工作模式清零*/
    ADC1->CR1 |= 0<<16;           /*独立工作模式*/
    ADC1->CR1 &= ~(1<<8);         /*非扫描模式*/
    ADC1->CR2 &= ~(1<<1);         /*单次转换模式*/
    ADC1->CR2 &= ~(7<<17);
    ADC1->CR2 |= 7<<17;           /*软件控制转换*/
    ADC1->CR2 |= 1<<20;           /*使用用外部触发(SWSTART)!!!	必须使用一个事件来触发*/
    ADC1->CR2 &= ~(1<<11);        /*右对齐*/
    ADC1->CR2 |= 1<<23;           /*使能温度传感器*/
    ADC1->SQR1 &= ~(0XF<<20);
    ADC1->SQR1 &= 0<<20;          /*1个转换在规则序列中 也就是只转换规则序列1*/

    /*设置通道1的采样时间*/
    ADC1->SMPR2 &= ~(7<<3);       /*通道1采样时间清空*/
    ADC1->SMPR2 |= 7<<3;          /*通道1  239.5周期,提高采样时间可以提高精确度*/

    ADC1->SMPR1 &= ~(7<<18);      /*清除通道16原来的设置*/
    ADC1->SMPR1 |= 7<<18;         /*通道16  239.5周期,提高采样时间可以提高精确度*/

    ADC1->CR2 |= 1<<0;            /*开启AD转换器*/
    ADC1->CR2 |= 1<<3;            /*使能复位校准*/
    while (ADC1->CR2 & (1<<3));   /*等待校准结束，该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除*/

    ADC1->CR2 |= 1<<2;            /*开启AD校准*/
    while (ADC1->CR2 & (1<<2));   /*等待校准结束，该位由软件设置以开始校准，并在校准结束时由硬件清除*/

    Battery.TestVal          = 3.95;  /*单位为v 电池实际电压  校准电压时修改*/
    Battery.ADInputVal       = 1.98;  /*单位为v R15和R17连接处电压 校准电压时修改*/
    Battery.ADRefVal         = 3.26;  /*单位为v 单片机供电电压   校准电压时修改*/
    Battery.Factor           = Battery.TestVal / Battery.ADInputVal;  /*计算电压计算系数*/
    Battery.OverDischargeCnt = 0;
    
    printf("Batter voltage AD init ...\r\n");  
}

/*获得ADC值*/
/*ch:通道值 0~16*/
/*返回值:转换结果*/
u16 Get_ADC(u8 ch)
{
    /*设置转换序列*/
    ADC1->SQR3 &= 0XFFFFFFE0;      /*规则序列1 通道ch*/
    ADC1->SQR3 |= ch;
    ADC1->CR2  |= 1<<22;           /*启动规则转换通道*/
    while (!(ADC1->SR & (1<<1)));  /*等待转换结束*/
    return ADC1->DR;               /*返回adc值*/
}

/*获取通道ch的转换值，取times次,然后平均*/
/*ch:通道编号*/
/*times:获取次数*/
/*返回值:通道ch的times次转换结果平均值*/
u16 Get_ADC_Average(u8 ch, u8 times)
{
    u32 temp_val = 0;

    u8 i;
    for (i = 0; i < times; i++)
    {
        temp_val += Get_ADC(ch);
    }
    return temp_val / times;
}
