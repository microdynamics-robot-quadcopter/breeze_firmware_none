#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_system_ms5611.h"
#include "stm32f10x_it.h"
#include "math.h"

#include "stm32f10x_driver_usart.h" /*debug*/
#include "stdio.h"

#define MS5611_Press_OSR MS5611_OSR_4096  /*气压采样精度*/
#define MS5611_Temp_OSR  MS5611_OSR_4096  /*温度采样精度*/

#define StartConvertTemp  0x01  /*开始转换温度*/
#define ConvertTemping    0x02  /*正在转换温度*/
#define StartConvertPress 0x03  /*开始转换气压*/
#define ConvertPressing   0x04  /*正在转换气压*/

#define BUFFER_SIZE       10

static uint8_t  CurState = StartConvertTemp;     /*当前状态*/
static uint32_t CurDelay = 0;                    /*转换延迟时间:us*/
static uint16_t PROM_C[MS5611_PROM_REG_COUNT];   /*标定参数存放*/
static uint32_t StartConvertTime;                /*启动转换时的时间:us*/
static int32_t TempCache;

static float AltOffsetM = 0;

#define PA_OFFSET_INIT_NUM 50

static float AltOffsetPa = 0;  /*存放着0米(离起飞所在平面)时对应的气压值, 这个值存放上电时的气压值*/
double PaOffsetNum       = 0;
uint16_t PaInitCnt       = 0;
uint8_t PaOffsetInited   = 0;
uint8_t Baro_Alt_Updated = 0;  /*气压计高度更新完成标志*/

/*units (Celsius degrees*100, mbar*100 )*/
/*单位  [温度 度] [气压 帕]  [高度 米]  */
volatile float MS5611_Pressure;
volatile float MS5611_Altitude;
volatile float MS5611_Temperature;

/*延时表:us 不同的采样精度对应不同的延时值*/
uint32_t MS5611_Delay_Us[9] = {
    1500,  /*MS5611_OSR_256  0.9ms  0x00*/
    1500,  /*MS5611_OSR_256  0.9ms      */
    2000,  /*MS5611_OSR_512  1.2ms  0x02*/
    2000,  /*MS5611_OSR_512  1.2ms      */
    3000,  /*MS5611_OSR_1024 2.3ms  0x04*/
    3000,  /*MS5611_OSR_1024 2.3ms      */
    5000,  /*MS5611_OSR_2048 4.6ms  0x06*/
    5000,  /*MS5611_OSR_2048 4.6ms      */
    11000, /*MS5611_OSR_4096 9.1ms  0x08*/
};

/*数据队列*/
static float TempBuffer[BUFFER_SIZE];
static float PressBuffer[BUFFER_SIZE];
static float AltBuffer[BUFFER_SIZE];

/*队列指针*/
static uint8_t temp_ptr  = 0;
static uint8_t press_ptr = 0;

void MS5611_TempPush(float val)   /*添加一个新的值到温度队列进行滤波*/
{
    TempBuffer[temp_ptr] = val;
    temp_ptr = (temp_ptr + 1) % BUFFER_SIZE;
}

void MS5611_PressPush(float val)  /*添加一个新的值到气压队列进行滤波*/
{
    PressBuffer[press_ptr] = val;
    press_ptr = (press_ptr + 1) % BUFFER_SIZE;
}

void MS5611_AltPush(float val)    /*添加一个新的值到高度队列进行滤波*/
{
    int16_t i;
    for (i = 1; i < BUFFER_SIZE; i++)
    {
        AltBuffer[i-1] = AltBuffer[i];
    }
    AltBuffer[BUFFER_SIZE-1] = val;
}

float MS5611_GetAvg(float *buff, int size)  /*读取队列的平均值*/
{
    float sum = 0.0;
    int i;
    for (i = 0; i < size; i++)
    {
        sum += buff[i];
    }
    return sum / size;
}

/**************************实现函数********************************************
*函数原型:      void MS5611_ReadPROM(void)
*功    能:      读取MS561101B的工厂标定值
读取气压计的标定值用于修正温度和气压的读数
*******************************************************************************/
void MS5611_ReadPROM(void)
{
    u8 inth, intl;
    int i;
    for (i = 0; i < MS5611_PROM_REG_COUNT; i++)
    {
        IIC_Start();
        IIC_SendByte(MS5611_ADDR);
        IIC_WaitAck();
        IIC_SendByte(MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));
        IIC_WaitAck();
        IIC_Stop();
        delay_us(5);
        IIC_Start();
        IIC_SendByte(MS5611_ADDR + 1);  /*进入接收模式*/
        delay_us(1);
        IIC_WaitAck();
        inth = IIC_ReadByte(1);         /*带ACK的读数据*/
        delay_us(1);
        intl = IIC_ReadByte(0);         /*最后一个字节NACK*/
        IIC_Stop();
        PROM_C[i] = (((uint16_t)inth << 8) | intl);
    }
}

/**************************实现函数********************************************
*函数原型:      void MS5611_Reset(void)
*功    能:      发送复位命令到MS561101B
*******************************************************************************/
void MS5611_Reset(void)
{
    IIC_Start();
    IIC_SendByte(MS5611_ADDR);   /*写地址*/
    IIC_WaitAck();
    IIC_SendByte(MS5611_RESET);  /*发送复位命令*/
    IIC_WaitAck();
    IIC_Stop();
}

/**************************实现函数********************************************
*函数原型:      void MS5611_StartConversion(uint8_t cmd)
*功    能:      发送启动转换命令到MS561101B
*cmd:           MS5611_D1  转换气压
                MS5611_D2  转换温度
*******************************************************************************/
void MS5611_StartConversion(uint8_t cmd)
{
    IIC_Start();
    IIC_SendByte(MS5611_ADDR);  /*写地址*/
    IIC_WaitAck();
    IIC_SendByte(cmd);          /*写转换命令*/
    IIC_WaitAck();
    IIC_Stop();
}

#define CMD_ADC_READ 0x00

/**************************实现函数********************************************
*函数原型:      uint32_t MS5611_GetConversion(void)
*功    能:      读取MS561101B的转换结果	 
*******************************************************************************/
uint32_t MS5611_GetConversion(void)
{
    uint32_t res = 0;
    u8 temp[3];
    
    IIC_Start();
    IIC_SendByte(MS5611_ADDR);      /*写地址*/
    IIC_WaitAck();
    IIC_SendByte(0);                /*start read sequence*/
    IIC_WaitAck();
    IIC_Stop();
    
    IIC_Start();
    IIC_SendByte(MS5611_ADDR + 1);  /*进入接收模式*/
    IIC_WaitAck();
    temp[0] = IIC_ReadByte(1);      /*带ACK的读数据  bit 23-16*/
    temp[1] = IIC_ReadByte(1);      /*带ACK的读数据  bit 8-15*/
    temp[2] = IIC_ReadByte(0);      /*带NACK的读数据 bit 0-7*/
    IIC_Stop();
    res = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return res;
}

/**************************实现函数********************************************
*函数原型:      void MS5611_init(void)
*功    能:	    初始化MS561101B
*******************************************************************************/
void MS5611_Init(void)
{
    MS5611_Reset();
    delay_ms(100);
    MS5611_ReadPROM();
}

/**************************实现函数********************************************
*函数原型:      void MS5611_GetTemperature(void)
*功    能:      读取温度转换结果
*******************************************************************************/
void MS5611_GetTemperature(void)
{
    TempCache = MS5611_GetConversion();
}

/**************************实现函数********************************************
*函数原型:      float MS5611_GetAltitude(void)
*功    能:      将当前的气压值转成高度
*******************************************************************************/
float MS5611_GetAltitude(void)
{
    static float Altitude;
    if (AltOffsetPa == 0)  /*是否初始化过0米气压值*/
    {
        if (PaInitCnt > PA_OFFSET_INIT_NUM) /*用PA_OFFSET_INIT_NUM次的平均值作为高度偏差值*/
        {
            AltOffsetPa = PaOffsetNum / PaInitCnt;
            PaOffsetInited = 1;
        }
        else
        {
            PaOffsetNum += MS5611_Pressure;
        }
        PaInitCnt++;
        Altitude = 0;    /*高度为0*/
        return Altitude;
    }
    Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / AltOffsetPa), 0.1903)) * 0.01f;  /*计算相对于上电时的位置的高度值单位为m*/
	Altitude = Altitude + AltOffsetM ;  /*加偏置*/
    return Altitude;
}

/**************************实现函数********************************************
*函数原型:      void MS5611_GetPressure(void)
*功    能:      读取气压转换结果并做补偿修正
*******************************************************************************/
void MS5611_GetPressure(void)
{
    int64_t off, sens;
    int64_t TEMP, T2, Aux_64, OFF2, SENS2;  /*64 bits*/
    int32_t RawPress = MS5611_GetConversion();
    int64_t dT = TempCache - (((int32_t)PROM_C[4]) << 8);

    TEMP = 2000 + (dT * (int64_t)PROM_C[5]) / 8388608;
    off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
    sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);

    if (TEMP < 2000)  /*second order temperature compensation*/
	{
        T2 = (((int64_t)dT) * dT) >> 31;
        Aux_64 = (TEMP - 2000) * (TEMP - 2000);
        OFF2   = (5 * Aux_64) >> 1;
        SENS2  = (5 * Aux_64) >> 2;
        TEMP   = TEMP - T2;
        off    = off - OFF2;
        sens   = sens - SENS2;
    }

    /*原始的方法*/
    MS5611_Pressure = (((((int64_t)RawPress) * sens) >> 21) - off) / 32768;

    /*温度队列处理*/
    MS5611_TempPush(TEMP * 0.01f);
    MS5611_Temperature = MS5611_GetAvg(TempBuffer, BUFFER_SIZE);  /*0.01c*/
    MS5611_Altitude = MS5611_GetAltitude();                       /*m*/
}

/**************************实现函数********************************************
*函数原型:      void MS5611_Thread(void)
*功    能:      MS5611BA的运行程序，需要定期调用以更新气压值和温度值
*******************************************************************************/
void MS5611_Thread(void)
{
    switch (CurState)
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
                MS5611_GetTemperature();                               /*取温度*/
                MS5611_StartConversion(MS5611_D1 + MS5611_Press_OSR);  /*启动气压转换*/
                CurDelay = MS5611_Delay_Us[MS5611_Press_OSR];          /*转换时间*/
                StartConvertTime = micros();                           /*计时开始*/
                CurState = ConvertPressing;                            /*下一个状态*/
            }
        break;

        case ConvertPressing:
            if ((micros() - StartConvertTime) > CurDelay)
            {
                MS5611_GetPressure();                                 /*更新计算*/
                Baro_Alt_Updated = 0xFF;                              /*高度更新完成*/
                MS5611_StartConversion(MS5611_D2 + MS5611_Temp_OSR);  /*开启温度转换*/
                CurDelay = MS5611_Delay_Us[MS5611_Temp_OSR];          /*转换时间*/
                StartConvertTime = micros();                          /*计时开始*/
                CurState = ConvertTemping;                            /*下一个状态*/
            }
        break;

        default:
            CurState = ConvertTemping;
        break;
    }
}

uint8_t MS5611_WaitBaroInitOffset(void)
{
    uint32_t now = 0;
    uint32_t starttime = 0;
    starttime = micros();   /*us*/

    while (!PaOffsetInited)
    {
        //printf("This is PaOffsetInited: %d", PaOffsetInited);  /*debug*/
        //printf("  This is now: %d", now);
        MS5611_Thread();
        now = micros();
        if ((now - starttime) / 1000 >= PA_OFFSET_INIT_NUM * 50)  /*超时*/
        {
            return 0;
        }
    }
    return 1;
}
