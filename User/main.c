#include "stm32f10x_driver_sys.h"
#include "stm32f10x_driver_pwm.h"
#include "stm32f10x_system_led.h"
#include "stm32f10x_system_delay.h"
#include "stm32f10x_system_battery.h"

void Hardware_Init(void)
{
    delay_init();
	led_init();
    PWM_Init();
    //Battery_Check_Init();
}

int main(void)
{
    Hardware_Init();   
    // PWM_test();
    led_test(1);
    while (1)
    {
        led_test(1);
        delay_ms(1000);
        led_test(0);
        delay_ms(1000);
    }
}
