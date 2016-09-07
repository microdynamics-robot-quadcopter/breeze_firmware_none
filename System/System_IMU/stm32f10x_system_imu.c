#include "stm32f10x_system_imu.h"
#include "stm32f10x_system_mpu6050.h"
#include "stm32f10x_algorithm_filter.h"
#include "stm32f10x_it.h"

imu_t imu = {0};
uint8_t imuCaliFlag = 0;

void IMU_Init(void)
{
    imu.ready    = 0;
    imu.caliPass = 1;
    LPF2pSetCutOffFreq_1(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutOffFreq_2(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutOffFreq_3(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutOffFreq_4(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutOffFreq_5(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
    LPF2pSetCutOffFreq_6(IMU_SAMPLE_RATE, IMU_FILTER_CUTOFF_FREQ);
}

/*should place to a level surface and keep it stop for 1~2 second*/
/*return 1 when finish*/
uint8_t IMU_Calibrate(void)
{
    static float AccSum[3]     = {0, 0, 0};
    static float GyroSum[3]    = {0, 0, 0};
    static uint16_t cnt        = 0;
    static uint16_t PreTime    = 0;
    static uint8_t calibrating = 0;
    uint8_t ret                = 0;
    uint8_t i                  = 0;
    uint16_t dt                = 0;
    uint16_t NowTime           = 0;

    NowTime = millis();
    dt = NowTime - PreTime;
    
    if (!calibrating)
    {
        calibrating = 1;
        for (i = 0; i < 3; i++)
        {
            cnt        = 0;
            imu.ready  = 0;
            AccSum[i]  = 0;
            GyroSum[i] = 0;
        }
    }
    
    if (dt >= 10)
    {
        if (cnt < 300)
        {
            for (i = 0; i < 3; i++)
            {
                AccSum[i]  += imu.accRaw[i];
                GyroSum[i] += imu.gyroRaw[i];
            }
            cnt++;
            PreTime = NowTime;
        }
        else
        {
            for (i = 0; i < 3; i++)
            {
                imu.accOffset[i]  = AccSum[i] / (float) cnt;
                imu.gyroOffset[i] = GyroSum[i] / (float) cnt;
            }
            imu.accOffset[2] -= CONSTANTS_ONE_G;
            calibrating = 0;
            ret = 1;
        }
    }
    
    return ret;
    
}

#define SENSOR_MAX_G 8.0f
#define SENSOR_MAX_W 2000.0f
#define ACC_SCALE (SENSOR_MAX_G / 32768.0f)
#define GYRO_SCALE (SENSOR_MAX_W / 32768.0f)

void IMU_ReadSensorHandle(void)
{
    uint8_t i;
    MPU6050_ReadAcc(imu.accADC);
    MPU6050_ReadGyro(imu.gyroADC);
    
    for (i = 0; i < 3; i++)
    {
        imu.accRaw[i] = (float) imu.accADC[i] * ACC_SCALE * CONSTANTS_ONE_G;
        imu.gyroRaw[i] = (float) imu.gyroADC[i] * GYRO_SCALE * M_PI_F / 180.0f;
    }
    
    imu.accb[0] = LPF2pApply_1(imu.accRaw[0]);
    imu.accb[1] = LPF2pApply_2(imu.accRaw[1]);
    imu.accb[2] = LPF2pApply_3(imu.accRaw[2]);
    imu.gyro[0] = LPF2pApply_4(imu.gyroRaw[0]);
    imu.gyro[1] = LPF2pApply_5(imu.gyroRaw[1]);
    imu.gyro[2] = LPF2pApply_6(imu.gyroRaw[2]);
}

#define ACCZ_ERR_MAX 0.05
#define CHECK_CNT    5

uint8_t IMU_Check(void)
{
    uint32_t AccZSum = 0;
    float AccZb = 0;
    uint8_t i;
    
    for (i = 0; i < CHECK_CNT; i++)
    {
        MPU6050_ReadAcc(imu.accADC);
        AccZSum += imu.accADC[2];
    }
    imu.accRaw[2] = (float) (AccZSum / (float) CHECK_CNT) * ACC_SCALE * CONSTANTS_ONE_G;
    AccZb = imu.accRaw[2] - imu.accOffset[2];
    
    if ((AccZb > CONSTANTS_ONE_G - ACCZ_ERR_MAX) && (AccZb < CONSTANTS_ONE_G + ACCZ_ERR_MAX))
    {
        imu.caliPass = 1;
    }
    else
    {
        imu.caliPass = 0;
    }
    return imu.caliPass;
}

