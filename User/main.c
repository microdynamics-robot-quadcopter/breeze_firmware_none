#include "stm32f10x_driver_sys.h"
#include "stm32f10x_system_delay.h"
#include "usart.h"
#include "stm32f10x_driver_led.h"

int main(void)
{
    delay_init();
    LED_Init();
    
    while (1)
    {
        LED0 = 0;
        delay_ms(1000);
        LED0 = 1;
        delay_ms(1000);
    }
}

