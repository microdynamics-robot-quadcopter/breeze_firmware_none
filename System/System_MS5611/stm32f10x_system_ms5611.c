#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_system_ms5611.h"
#include "stm32f10x_it.h"
#include "math.h"

#define MS5611_Press_OSR MS5611_OSR_4096
#define MS5611_Temp_OSR  MS5611_OSR_4096

#define StartConvertTemp  0x01
#define ConvertTemping    0x02
#define StartConvertPress 0x03
#define ConvertPressing   0x04

#define BUFFER_SIZE       10

static uint8_t  CurState = StartConvertTemp;
static uint32_t CurDelay = 0;
static uint16_t PROM_C[MS5611_PROM_REG_COUNT];
static uint32_t StartConvertTime;
static int32_t TempCache;

static float AltOffsetM = 0;

#define PA_OFFSET_INIT_NUM 50

static float AltOffsetPa = 0;
double PaOffsetNum = 0;
uint16_t PaInitCnt = 0;
uint8_t PaOffsetInited = 0;

uint8_t Baro_Alt_Updated = 0;

volatile float MS5611_Pressure;
volatile float MS5611_Altitude;
volatile float MS5611_Temperature;

// 延时表单位 us 	  不同的采样精度对应不同的延时值
uint32_t MS5611_Delay_Us[9] = {
    1500,//MS561101BA_OSR_256 0.9ms  0x00
    1500,//MS561101BA_OSR_256 0.9ms  
    2000,//MS561101BA_OSR_512 1.2ms  0x02
    2000,//MS561101BA_OSR_512 1.2ms
    3000,//MS561101BA_OSR_1024 2.3ms 0x04
    3000,//MS561101BA_OSR_1024 2.3ms
    5000,//MS561101BA_OSR_2048 4.6ms 0x06
    5000,//MS561101BA_OSR_2048 4.6ms
    11000,//MS561101BA_OSR_4096 9.1ms 0x08
};

static float TempBuffer[BUFFER_SIZE];
static float PressBuffer[BUFFER_SIZE];
static float AltBuffer[BUFFER_SIZE];

static uint8_t temp_ptr = 0;
static uint8_t press_ptr = 0;

void MS5611_TempPush(float val)
{
    TempBuffer[temp_ptr] = val;
    temp_ptr = (temp_ptr + 1) % BUFFER_SIZE;
}

void MS5611_PressPush(float val)
{
    PressBuffer[press_ptr] = val;
    press_ptr = (press_ptr + 1) % BUFFER_SIZE;
}

void MS5611_AltPush(float val)
{
    int16_t i;
    for (i =1; i < BUFFER_SIZE; i++)
    {
        AltBuffer[i-1] = AltBuffer[i];
    }
    AltBuffer[BUFFER_SIZE-1] = val;
}

float MS5611_GetAvg(float *buff, int size)
{
    int i;
    float sum = 0.0;
    for (i = 0; i < size; i++)
    {
        sum += buff[i];
    }
    return sum / size;
}

void MS5611_ReadPROM(void)
{
    u8 inth, intl;
    int i;
    for (i = 0; i < MS5611_PROM_REG_COUNT; i++)
    {
        IIC_Start();
        IIC_Send_Byte(MS5611_ADDR);
        IIC_Wait_Ack();
        IIC_Send_Byte(MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));
        IIC_Wait_Ack();
        IIC_Stop();
        delay_us(5);
        IIC_Start();
        IIC_Send_Byte(MS5611_ADDR + 1);
        delay_us(1);
        IIC_Wait_Ack();
        inth = IIC_Read_Byte(1);
        delay_us(1);
        intl = IIC_Read_Byte(0);
        IIC_Stop();
        PROM_C[i] = (((uint16_t)inth << 8) | intl);
    }
}

void MS5611_Reset(void)
{
    IIC_Start();
    IIC_Send_Byte(MS5611_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(MS5611_RESET);
    IIC_Wait_Ack();
    IIC_Stop();
}

void MS5611_StartConversion(uint8_t cmd)
{
    IIC_Start();
    IIC_Send_Byte(MS5611_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(cmd);
    IIC_Wait_Ack();
    IIC_Stop();
}

#define CMD_ADC_READ 0x00

uint32_t MS5611_GetConversion(void)
{
    uint32_t res = 0;
    u8 temp[3];
    IIC_Start();
    IIC_Send_Byte(MS5611_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(0);
    IIC_Wait_Ack();
    IIC_Stop();
    
    IIC_Start();
    IIC_Send_Byte(MS5611_ADDR + 1);
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1);
    temp[1] = IIC_Read_Byte(1);
    temp[2] = IIC_Read_Byte(0);
    IIC_Stop();
    res = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return res;
}

void MS5611_Init(void)
{
    MS5611_Reset();
    delay_ms(100);
    MS5611_ReadPROM();
}

void MS5611_GetTemperature(void)
{
    TempCache = MS5611_GetConversion();
}

float MS5611_GetAltitude(void)
{
    static float Altitude;
    if (AltOffsetPa == 0)
    {
        if (PaInitCnt > PA_OFFSET_INIT_NUM)
        {
            AltOffsetPa = PaOffsetNum / PaInitCnt;
            PaOffsetInited = 1;
        }
        else
        {
            PaOffsetNum += MS5611_Pressure;
        }
        PaInitCnt++;
        Altitude = 0;
        return Altitude;
    }
    Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / AltOffsetPa), 0.1903)) * 0.01f;
	Altitude = Altitude + AltOffsetM ;  //加偏置
    return Altitude;
}

void MS5611_GetPressure(void)
{
    int64_t off, sens;
    int64_t TEMP, T2, Aux_64, OFF2, SENS2;  // 64 bits
    int32_t rawPress = MS5611_GetConversion();
    int64_t dT = TempCache - (((int32_t)PROM_C[4]) << 8);

    TEMP = 2000 + (dT * (int64_t)PROM_C[5]) / 8388608;
    off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
    sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);

    if (TEMP < 2000)
	{   // second order temperature compensation
        T2 = (((int64_t)dT) * dT) >> 31;
        Aux_64 = (TEMP - 2000) * (TEMP - 2000);
        OFF2 = (5 * Aux_64) >> 1;
        SENS2 = (5 * Aux_64) >> 2;
        TEMP = TEMP - T2;
        off = off - OFF2;
        sens = sens - SENS2;
    }

    //原始的方法
    MS5611_Pressure = (((((int64_t)rawPress) * sens) >> 21) - off) / 32768;

    //温度队列处理
    MS5611_TempPush(TEMP * 0.01f);
    MS5611_Temperature = MS5611_GetAvg(TempBuffer, BUFFER_SIZE); //0.01c
    MS5611_Altitude = MS5611_GetAltitude(); // 单位：m 	
}

void MS5611_Thread(void)
{
    switch(CurState)
    {
        case StartConvertTemp:
            MS5611_StartConversion(MS5611_D2 + MS5611_Temp_OSR);
            CurDelay = MS5611_Delay_Us[MS5611_Temp_OSR];
            StartConvertTime = micros();
            CurState = ConvertTemping;
        break;
        
        case ConvertTemping:
            if ((micros() - StartConvertTime) > CurDelay)
            {
                MS5611_GetTemperature();
                CurState = StartConvertPress;
            }
        break;
            
        case StartConvertPress:
            MS5611_StartConversion(MS5611_D1 + MS5611_Press_OSR);
            CurDelay = MS5611_Delay_Us[MS5611_Press_OSR];
            StartConvertTime = micros();
            CurState = ConvertPressing;
        break;
        
        case ConvertPressing:
            if ((micros() - StartConvertTime) > CurDelay)
            {
                MS5611_GetPressure();
                Baro_Alt_Updated = 0xFF;
                CurState = StartConvertTemp;
            }
        break;
        
        default:
            CurState = StartConvertTemp;
        break;
    }
}

uint8_t WaitBaroInitOffset(void)
{
    uint32_t now = 0;
    uint32_t starttime = 0;
    starttime = micros();
    
    while(!PaOffsetInited)
    {
        MS5611_Thread();
        now = micros();
        if ((now - starttime) / 1000 == PA_OFFSET_INIT_NUM * 50)
        {
            return 0;
        }
    }
    return 1;
}