#include "stm32f10x_it.h"
#include "stm32f10x_driver_clock.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_driver_flash.h"
#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_nvic.h"
#include "stm32f10x_driver_io.h"
#include "stm32f10x_driver_timer.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_battery.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_motor.h"
#include "stm32f10x_module_mpu6050.h"
#include "stm32f10x_module_ms5611.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_algorithm_altitude.h"
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_algorithm_filter.h"
#include "stm32f10x_algorithm_flight.h"
#include "stm32f10x_algorithm_imu.h"

static u16 battery_check_count = 0;

void Hardware_Init(void)
{
    Clock_Init();
    Delay_Init();
    USART_InitUSART(115200);
    Timer_InitTIM4(1000, clock_system);
    Flash_Unlock();
    EEPROM_LoadParamsFromEEPROM();
    LED_Init();
    Motor_Init();
    Battery_Init();
    IIC_Init();
    MPU6050_Init();
    NRF24L01_Init();
    Battery_Check();
    MS5611_Init();
    IMU_Init();
    MS5611_WaitBaroInitOffset();
    control_altitude_mode = CONTROL_STATE_MANUAL;
}

int main(void)
{
    Hardware_Init();
    LED_SetInitialLight();

    LED_A_ON;

    while(1)
    {
        if (timer_loop_flag_100hz)
        {
            timer_loop_flag_100hz = false;
            IMU_StartSO3Thread();
            altitude_acc_update_flag = true;
            MS5611_UpdateData();
            if (imu_cali_flag)
            {
                if (IMU_Calibrate())
                {
                    imu_cali_flag                = false;
                    eeprom_params_request_flag   = true;
                    IMU_TableStructure.flag_cali = true;
                    LED_B_ON;
                }
            }
            Control_CallPIDAngleRate();
            Control_SetMotorPWM();
        }

        NRF24L01_IRQHandler();

        if (timer_loop_flag_50hz)
        {
            timer_loop_flag_50hz = false;
            CommLink_ProcessDataFromNRF();
            Flight_SetMode();
            Altitude_CombineData();
            Control_SetAltitude();
            Control_CallPIDAngle();
            CommLink_WriteDebugData();
        }
        if (timer_loop_flag_10hz)
        {
            timer_loop_flag_10hz = false;
            if ((++battery_check_count) * 100 >= BATTERY_CHECK_PERIOD)
            {
                battery_check_count = 0;
                Battery_Check();
                if (Battery_InformationStructure.flag_alarm)
                {
                    LED_D_ON;
                }
            }
            if (eeprom_params_request_flag)
            {
                eeprom_params_request_flag = false;
                EEPROM_SaveParamsToEEPROM();
            }
        }
    }
}
