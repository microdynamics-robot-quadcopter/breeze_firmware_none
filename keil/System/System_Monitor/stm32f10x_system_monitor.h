#ifndef __STM32F10X_SYSTEM_MONITOR__
#define __STM32F10X_SYSTEM_MONITOR__

#include "stm32f10x.h"

#define UartSendInt16(_x) UartSendBuffer((uint8_t *)(&_x), 2)
#define UartSendInt32(_x) UartSendBuffer((uint8_t *)(&_x), 4)

/*only send data*/
typedef union int16un
{
    uint8_t b[2];
    int16_t val;
}int16_un;

typedef union int32un
{
    uint8_t b[4];
    int32_t val;
}int32_un;

typedef struct HawkerPacket_tt
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t len;
    uint8_t sum;

    int16_un roll;
    int16_un pitch;
    int16_un yaw;
    int32_un alti;
    int16_un temp;
    int32_un pres;
    int16_un speed;
}HawkerPacket_t;

typedef struct DataPackage_tt
{
    uint8_t header[2];
    uint8_t cmd;
    uint8_t len;
    uint8_t data[30];
    uint8_t sum;
}DataPackage_t;

extern HawkerPacket_t up;
extern uint8_t pcCmdFlag;
extern uint16_t rcData[4];

static void EndianConvert(uint8_t arr[], uint8_t len);

extern void CommPCUploadHandle(void);
extern void CommPCProcessCmd(void);
extern void CommPC(uint8_t c);
extern void testCommPC(void);
extern void ReturnPIDHead(uint8_t pidType);

#endif
