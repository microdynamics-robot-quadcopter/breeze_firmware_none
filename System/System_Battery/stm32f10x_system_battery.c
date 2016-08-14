#include "stm32f10x_system_battery.h"
#include "stdio.h"

int FLY_ENABLE = 1; //test
//实例化一个电压信息结构体
Bat_TypedefStructure Battery;

//得到ADC采样内部温度传感器的温度值
//返回值3位温度值 XXX*0.1C	 
extern int Get_Temp(void)
{			
}

//返回电池电压AD值
extern int Get_Battery_AD(void)
{
    return Get_ADC_Average(8, 5);
}

//检测电池电压
extern void Battery_Check(void)
{
    Battery.BatteryADVal  = Get_Battery_AD();            //电池电压检测  
    Battery.BatteryRealVal = Battery.BatteryFactor * (Battery.BatteryADVal / 4096.0) * Battery.BatteryADRefVal;  //实际电压 值计算	
	
    if (FLY_ENABLE)
    {
        //处于电机开启等飞行状态，在过放电压值（BAT_OVERDIS_VAL）以上0.03v以上，开始报警
        if (Battery.BatteryRealVal <= (BAT_OVERDIS_VAL + 0.03))
        {
            Battery.BatteryAlarmFlag = 1;
        }
        else
        {
            Battery.BatteryAlarmFlag = 0;
        }
			
		//过放保护，Battery overdischarge protect
        if (Battery.BatteryRealVal <= BAT_OVERDIS_VAL)
        {
            Battery.BatteryOverDischargeCnt++;
            if (Battery.BatteryOverDischargeCnt > 8)
            {
				//altCtrlMode=LANDING;
				//rcData[0]=1500;rcData[1]=1500;rcData[2]=1500;rcData[3]=1500;
			}
		}
        else
        {
            Battery.BatteryOverDischargeCnt = 0;
        }
	}
    else
    {
        if ((Battery.BatteryRealVal < BAT_ALARM_VAL) && (Battery.BatteryRealVal > BAT_CHARGE_VAL))//低于3.7v 且大于充电检测电压 BAT_CHG_VAL
        {	
            Battery.BatteryAlarmFlag = 1;
        }
        else
        {
            Battery.BatteryAlarmFlag = 0;
        }
    }
		
    if (Battery.BatteryRealVal < BAT_CHARGE_VAL)  //on charge
    { 
        Battery.BatteryChargeState = 1; 
    }
    else
    {
        Battery.BatteryChargeState = 0;
    }
}

//初始化电池检测ADC
//开启ADC1的通道8	
//BatteryCheck---->PB0
extern void Battery_Check_Init(void)
{
    //先初PB0为模拟输入
    RCC->APB2ENR  |= 1<<3;        //使能PORTB口时钟 
    GPIOB->CRL    &= 0XFFFFFFF0;  //PB0	anolog输入


    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
    while (ADC1->CR2 & (1<<2));   //等待校准结束
  
    Battery.BatteryTestVal          = 3.95;  //单位为v 电池实际电压  校准电压时修改
    Battery.BatteryADInputVal       = 1.98;  //单位为v R15和R17连接处电压 校准电压时修改
    Battery.BatteryADRefVal         = 3.26;  //单位为v 单片机供电电压   校准电压时修改
    Battery.BatteryFactor           = Battery.BatteryTestVal / Battery.BatteryADInputVal;  //计算电压计算系数
    Battery.BatteryOverDischargeCnt = 0;
    
    printf("Batter voltage AD init ...\r\n");  
}

//获得ADC值
//ch:通道值 0~16
//返回值:转换结果
extern u16 Get_ADC(u8 ch)
{
    //设置转换序列	  		 
}

//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
extern u16 Get_ADC_Average(u8 ch, u8 times)
{
    u8 i;
    u32 temp_val = 0;
}
