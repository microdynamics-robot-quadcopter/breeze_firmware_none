#ifndef __STM32F10X_ALGORITHM_BAR_H__
#define __STM32F10X_ALGORITHM_BAR_H__

#include "stm32f10x.h"

/*us store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
#define ALT_THREAD_RPD 5000

typedef struct NAV_TT
{
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
}nav_t;

extern nav_t nav;
extern float z_est[3];
extern uint8_t landed;
extern uint8_t accUpdated;

static void Intertial_Filter_Predict(float dt, float x[3]);
static void Intertial_Filter_Corrent(float e, float dt, float x[3], int i, float w);

extern void AltitudeCombineThread(void);

#endif
