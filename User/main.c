#include "stm32f10x_driver_sys.h"
#include "stm32f10x_driver_pwm.h"
#include "stm32f10x_driver_tim.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_system_led.h"
#include "stm32f10x_system_battery.h"

void Hardware_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
	led_init();
    PWM_Init();
    usart_init(115200);
    TIM4_Init(4999, 7199);
    //Battery_Check_Init();
}

int main(void)
{
//    u8 len = 0;
//    u8 i;
//    u16 time = 0;
    Hardware_Init();   
    PWM_MotorFlash(200, 200, 200, 200);
    while (1)
    {
        led_test(1);
    }
    
//    while (1)
//    {
//        if (loop100HZCnt >= 10)
//        {
//            loop100HZCnt = 0;
//        }
//        
//    }
    
//    while (1)
//    {
//        led_test(1);
//        delay_ms(1000);
//        led_test(0);
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
