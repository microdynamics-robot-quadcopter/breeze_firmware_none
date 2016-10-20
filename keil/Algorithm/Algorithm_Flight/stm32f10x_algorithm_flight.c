#include "stm32f10x_system_rpdata.h"
#include "stm32f10x_algorithm_bar.h"
#include "stm32f10x_algorithm_flight.h"
#include "stm32f10x_algorithm_control.h"

/*for copter launch and flight mode switch.  it's raw and simple for climb rate mode now*/
/*TOBE IMPROVED*/
void FlightStateSet(void)
{
    if (FLY_ENABLE)
    {
        if (NRF_Data.throttle >= 600)
        {
            if (altCtrlMode != CLIMB_RATE)
            {
                zIntReset   = 1;
                thrustZSp   = 0;
                altCtrlMode = CLIMB_RATE;
                offLandFlag = 1;
                altLand     = -nav.z;    /*记录起飞时的高度*/
                SetHeadFree(1);
            }
        }
        else
        {
            if (altCtrlMode == MANUAL)
            {
                NRF_Data.throttle = 200; /*手动模式待机转200*/
            }
        }
    }
}