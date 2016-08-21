#ifndef __STM32F10X_DRIVER_EEPROM_H__
#define __STM32F10X_DRIVER_EEPROM_H__

#include "stm32f10x.h"

typedef struct EEPROM
{
    float pidPitch[3];
    float pidPitchRate[3];
    float pidYaw[3];
    float pidYawRate[3];
    float pidRoll[3];
    float pidRollRate[3];
    float pidAlt[3];
    float pidAltVel[3];
    float accOffset[3];
    float gyroOffset[3];
    float magOffset[3];
    float NRFAddr[5];
    float version;
    float BTState;
    float NRFMatchFlag;
}EEPROM_Table;

extern EEPROM_Table table;
extern uint8_t gParamsSaveEEPROMRequest;

extern void LoadParamsFromEEPROM(void);
extern void ParamSetDefault(void);
extern void ParamToTable(void);
extern void TableToParam(void);
extern void TableWriteEEPROM(void);
extern void TableReadEEPROM(void);
extern void SaveParamsToEEPROM(void);

static uint8_t isEEPROMValid(void);

#endif
