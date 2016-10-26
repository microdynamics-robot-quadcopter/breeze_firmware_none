#include "stm32f10x_driver_sys.h"
#include "stm32f10x_driver_pwm.h"
#include "stm32f10x_driver_tim.h"
#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_flash.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_system_led.h"
#include "stm32f10x_system_rpdata.h"
#include "stm32f10x_system_ms5611.h"
#include "stm32f10x_system_battery.h"
#include "stm32f10x_system_mpu6050.h"
#include "stm32f10x_system_nrf24l01.h"
#include "stm32f10x_algorithm_bar.h"
#include "stm32f10x_algorithm_imu.h"
#include "stm32f10x_algorithm_flight.h"
#include "stm32f10x_algorithm_control.h"
#include "stm32f10x_it.h"

/*software counter*/
uint16_t BatCnt = 0;

void Hardware_Init(void)
{
    SystemClock_HSE(9);
    CycleCounter_Init();                     /*Init cycle counter*/
    SysTick_Config(SystemCoreClock / 1000);  /*SysTick开启系统tick定时器并初始化其中断，中断溢出时间为1ms，写法是固定的*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    usart_Init(115200);
    TIM4_Init(1000, SysClock);
    //FLASH_Unlock();
    STMFLASH_Unlock();
    LoadParamsFromEEPROM();
    //delay_init();
    LED_Init();
    PWM_Init();
    Battery_CheckInit();
    IIC_Init();
    MPU6050_Init();
    NRF24L01_Init();
    Battery_Check();
    MS5611_Init();
    IMU_Init();
    altCtrlMode = MANUAL;
    MS5611_WaitBaroInitOffset();
}

int main(void)
{
//    u8 len = 0;
//    u8 i;
//    u16 time = 0;
    Hardware_Init();
//  PWM_MotorFlash(200, 200, 200, 200);
   
//    while (1)
//    {
//        NRF_Irq();
//        ProcessDataFromNRF();
//    }

    LedA_On;
    while(1)
    {
        if (loop100HzFlag)
        {
            loop100HzFlag = 0;

            IMU_SO3Thread();
            accUpdated = 1;
            MS5611_Thread();

            if (imuCaliFlag)  /*为1表示进行校准*/
            {
                if (IMU_Calibrate())
                {
                    imuCaliFlag              = 0;
                    gParamsSaveEEPROMRequest = 1;
                    imu.caliPass             = 1;
                    LedB_On;
                }
            }
            ControlAttiRate();
            ControlMotor();
        }

        NRF_Irq();

        if (loop50HzFlag)
        {
            loop50HzFlag = 0;

            ProcessDataFromNRF();
            FlightStateSet();
            AltitudeCombineThread();
            ControlAlti();
            ControlAttiAng();
        }

        if (loop10HzFlag)
        {
            loop10HzFlag = 0;

            if ((++BatCnt) * 100 >= BAT_CHECK_PERIOD)
            {
                BatCnt = 0;
                Battery_Check();
            }

            if (gParamsSaveEEPROMRequest)
            {
                gParamsSaveEEPROMRequest = 0;
                SaveParamsToEEPROM();
            }
        }
    }
//    while (1)
//    {
//        LED_test(1);
//        delay_ms(1000);
//        LED_test(0);
//        delay_ms(1000);
//        if (USART_RX_STA & 0x8000)
//        {
//            len = USART_RX_STA & 0x3f;
//            printf("\n\nThis is the content you send!!!\n\n");
//            for (i = 0; i < len; i++)
//            {
//                USART_SendData(USART1, USART_RX_BUF[i]);
//                while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
//            }
//            printf("\n\n");
//            USART_RX_STA = 0;
//        }
//        else
//        {
//            time++;
//            if (time % 5000 == 0)
//            {
//                printf("I am supermaker!!!\n\n");
//            }
//            else if (time % 200 == 0)
//            {
//                printf("This is 200 numbers\n\n");
//            }
//            delay_ms(10);
//        }   
//    }
}
