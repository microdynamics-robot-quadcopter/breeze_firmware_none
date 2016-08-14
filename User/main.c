#include "stm32f10x_driver_sys.h"
#include "stm32f10x_system_delay.h"
#include "stm32f10x_system_led.h"

int main(void)
{
	led_init();
	led_test();
    while (1)
    {
        
    }
}

