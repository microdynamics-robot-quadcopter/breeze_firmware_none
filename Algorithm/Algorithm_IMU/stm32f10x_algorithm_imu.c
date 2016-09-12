#include "stm32f10x_system_mpu6050.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_filter.h"
#include "stm32f10x_it.h"
#include "math.h"

imu_t imu = {0};
uint8_t imuCaliFlag = 0;

void IMU_Init(void)
{
    imu.ready    = 0;  /*需要先校准陀螺*/
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
uint8_t IMU_Calibrate(void)  /*检测时间为3s*/
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
    float AccZb      = 0;
    
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

/*
in standard sequence , roll-pitch-yaw , x-y-z
angle in rad
get DCM for ground to body
*/
static void EularToDCM(float DCM[3][3], float pitch, float yaw, float roll)
{
    float cosx, cosy, cosz, sinx, siny, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(roll * M_PI_F / 180.0f);
    sinx = sinf(roll * M_PI_F / 1800.0f);
    cosy = cosf(pitch * M_PI_F / 180.0f);
    siny = sinf(pitch * M_PI_F / 180.0f);
    cosz = cosf(yaw * M_PI_F / 180.0f);
    sinz = sinf(yaw * M_PI_F / 180.0f);
    
    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = cosz * sinx;
    sinzsinx = sinz * sinx;
    
    DCM[0][0] = coszcosy;
    DCM[0][1] = cosy * sinz;
    DCM[0][2] = -siny;
    DCM[1][0] = -sinzcosx + (coszsinx * siny);
    DCM[1][1] = coszcosx + (sinzsinx * siny);
    DCM[1][2] = sinx * cosy;
    DCM[2][0] = (sinzsinx) + (coszcosx * siny);
    DCM[2][1] = -(coszsinx) + (sinzcosx * siny);
    DCM[2][2] = cosy * cosx;
}

/*Auxiliary variables to reduce number of repeated operations*/
static float q0 = 1.0f;
static float q1 = 0.0f;   /*quaternion of sensor frame relative to auxiliary frame*/
static float q2 = 0.0f;
static float q3 = 0.0f;

static float dq0 = 0.0f;
static float dq1 = 0.0f;  /*quaternion of sensor frame relative to auxiliary frame*/
static float dq2 = 0.0f;
static float dq3 = 0.0f;

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};  /*bias estimation*/
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;

static uint8_t bFilterInit = 0;

/*函数名：invSqrt(void)*/
/*描述：求平方根的倒数*/
/*该函数是经典的Carmack求平方根算法，效率极高，使用魔数0x5f375a86*/
static float InvSqrt(float num)
{
    volatile long  i;
    volatile float x;
    volatile float y;
    volatile const float f = 1.5f;
    
    x = num * 0.5f;
    y = num;
    i = *((long*) &y);
    i = 0x5F375A86 - (i >> 1);
    y = *((float*) &i);
    y = y * (f - (x * y * y));
    return y;
}

/*Using accelerometer, sense the gravity vector*/
/*Using magnetometer, sense yaw*/
static void NonLinearSO3AHRSInit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll  = cosf(initialRoll);
    sinRoll  = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll    = cosf(initialRoll * 0.5f);
    sinRoll    = sinf(initialRoll * 0.5f);

    cosPitch   = cosf(initialPitch * 0.5f);
    sinPitch   = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    /*auxillary variables to reduce number of repeated operations, for 1st pass*/
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

/*函数名：NonlinearSO3AHRSupdate()*/
/*描述：姿态解算融合，是Crazepony和核心算法*/
/*使用的是Mahony互补滤波算法，没有使用Kalman滤波算法*/
/*改算法是直接参考pixhawk飞控的算法，可以在Github上看到出处*/
/*https://github.com/hsteinhaus/PX4Firmware/blob/master/src/modules/attitude_estimator_so3/attitude_estimator_so3_main.cpp*/
static void NonLinearSO3AHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt) 
{
    float recipNorm;
    float halfex = 0.0f;
    float halfey = 0.0f;
    float halfez = 0.0f;

    /*Make filter converge to initial solution faster*/
    /*This function assumes you are in static position*/
    /*WARNING : in case air reboot, this can cause problem. But this is very unlikely happen*/
    if (bFilterInit == 0)
    {
        NonLinearSO3AHRSInit(ax,ay,az,mx,my,mz);
        bFilterInit = 1;
    }
    	
    /*! If magnetometer measurement is available, use it.*/
    if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
    {
        float hx, hy, hz, bx, bz;
        float halfwx, halfwy, halfwz;

        /*Normalise magnetometer measurement*/
        /*Will sqrt work better? PX4 system is powerful enough?*/
        recipNorm = InvSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /*Reference direction of Earth's magnetic field*/
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
        bx = sqrt(hx * hx + hy * hy);
        bz = hz;

        /*Estimated direction of magnetic field*/
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        /*Error is sum of cross product between estimated direction and measured direction of field vectors*/
        halfex += (my * halfwz - mz * halfwy);
        halfey += (mz * halfwx - mx * halfwz);
        halfez += (mx * halfwy - my * halfwx);
    }

    /*增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G*/
    /*Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)*/
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        float halfvx, halfvy, halfvz;

        /*Normalise accelerometer measurement*/
        /*归一化，得到单位加速度*/
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);

        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /*Estimated direction of gravity and magnetic field*/
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        /*Error is sum of cross product between estimated direction and measured direction of field vectors*/
        halfex += ay * halfvz - az * halfvy;
        halfey += az * halfvx - ax * halfvz;
        halfez += ax * halfvy - ay * halfvx;
    }

    /*Apply feedback only when valid data has been gathered from the accelerometer or magnetometer*/
    if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f)
    {
        /*Compute and apply integral feedback if enabled*/
        if (twoKi > 0.0f)
        {
            gyro_bias[0] += twoKi * halfex * dt;    /*integral error scaled by Ki*/
            gyro_bias[1] += twoKi * halfey * dt;
            gyro_bias[2] += twoKi * halfez * dt;

            /*apply integral feedback*/
            gx += gyro_bias[0];
            gy += gyro_bias[1];
            gz += gyro_bias[2];
        }
        else
        {
            gyro_bias[0] = 0.0f;    /*prevent integral windup*/
            gyro_bias[1] = 0.0f;
            gyro_bias[2] = 0.0f;
        }

        /*Apply proportional feedback*/
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    /*Time derivative of quaternion. q_dot = 0.5*q\otimes omega.*/
    /*! q_k = q_{k-1} + dt*\dot{q}*/
    /*! \dot{q} = 0.5*q \otimes P(\omega)*/
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx); 

    q0 += dt * dq0;
    q1 += dt * dq1;
    q2 += dt * dq2;
    q3 += dt * dq3;

    /*Normalise quaternion*/
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    /*Auxiliary variables to avoid repeated arithmetic*/
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

#define so3_comp_params_Kp  1.0f
#define so3_comp_params_Ki  0.05f


/*函数名：IMU_SO3Thread(void)*/
/*描述：姿态软件解算融合函数*/
/*该函数对姿态的融合是软件解算，Crazepony现在不使用DMP硬件解算*/
void IMU_SO3Thread(void)
{
    /*! Time constant*/
    float dt = 0.01f;                           /*s*/
    static uint32_t tPrev = 0, startTime = 0;   /*us*/
    uint32_t now;
    uint8_t i;

    /*output euler angles*/
    float euler[3] = {0.0f, 0.0f, 0.0f};        /*rad*/

    /*Initialization*/
    float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };  /**< init: identity matrix */
    float acc[3]        = {0.0f, 0.0f, 0.0f};   /*m/s^2*/
    float mag[3]        = {0.0f, 0.0f, 0.0f};
    float gyro[3]       = {0.0f, 0.0f, 0.0f};   /*rad/s*/

    /*need to calc gyro offset before imu start working*/
    static float gyro_offsets_sum[3] = {0.0f, 0.0f, 0.0f};  /*gyro_offsets[3] = {0.0f, 0.0f, 0.0f}*/
    static uint16_t offset_count = 0;

    now = micros();
    dt = (tPrev > 0) ? (now - tPrev) / 1000000.0f : 0;
    tPrev = now;

    IMU_ReadSensorHandle();

	if (!imu.ready)
    {
        if (startTime == 0)
        {
            startTime = now;
        }
		
        gyro_offsets_sum[0] += imu.gyroRaw[0];
        gyro_offsets_sum[1] += imu.gyroRaw[1];
        gyro_offsets_sum[2] += imu.gyroRaw[2];
        offset_count++;

        if (now > startTime + GYRO_CALC_TIME)
        {
            imu.gyroOffset[0]   = gyro_offsets_sum[0] / offset_count;
            imu.gyroOffset[1]   = gyro_offsets_sum[1] / offset_count;
            imu.gyroOffset[2]   = gyro_offsets_sum[2] / offset_count;
            offset_count        = 0;
            gyro_offsets_sum[0] = 0;
            gyro_offsets_sum[1] = 0;
            gyro_offsets_sum[2] = 0;

            imu.ready = 1;
            startTime = 0;
        }
        return;
    }

    gyro[0] = imu.gyro[0] - imu.gyroOffset[0];
    gyro[1] = imu.gyro[1] - imu.gyroOffset[1];
    gyro[2] = imu.gyro[2] - imu.gyroOffset[2];

    acc[0] = -imu.accb[0];
    acc[1] = -imu.accb[1];
    acc[2] = -imu.accb[2];

    /*NOTE : Accelerometer is reversed.*/
    /*Because proper mount of PX4 will give you a reversed accelerometer readings.*/
    NonLinearSO3AHRSUpdate(gyro[0], gyro[1], gyro[2], -acc[0], -acc[1], -acc[2], mag[0], mag[1], mag[2],
                           so3_comp_params_Kp, so3_comp_params_Ki, dt);

        /*Convert q->R, This R converts inertial frame to body frame.*/
        Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;  /*11*/
        Rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3);  /*12*/
        Rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2);  /*13*/
        Rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3);  /*21*/
        Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;  /*22*/
        Rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1);  /*23*/
        Rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2);  /*31*/
        Rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1);  /*32*/
        Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;  /*33*/

        /*1-2-3 Representation.*/
        /*Equation (290)*/
        /*Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.*/
        /*Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.*/
        euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);  /*! Roll*/
        euler[1] = -asinf(Rot_matrix[2]);                 /*! Pitch*/
        euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);

        /*DCM . ground to body*/
        for (i = 0; i < 9; i++)
        {
            *(&(imu.DCMgb[0][0]) + i) = Rot_matrix[i];
        }

        imu.rollRad  = euler[0];
        imu.pitchRad = euler[1];
        imu.yawRad   = euler[2];

        imu.roll  = euler[0] * 180.0f / M_PI_F;
        imu.pitch = euler[1] * 180.0f / M_PI_F;
        imu.yaw   = euler[2] * 180.0f / M_PI_F;
}
