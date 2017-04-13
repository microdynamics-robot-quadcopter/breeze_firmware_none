/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_mpu6050.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.04
Description: implement the mpu6050 function
Others:      none
Function List:
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.11  modify the module
*******************************************************************************/

#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_mpu6050.h"
#include "stdio.h"

uint8_t buffer[14];
int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset = 0;
int16_t Gy_offset = 0;
int16_t Gz_offset = 0;

/******************************************************************************
*Function prototype:  unsigned char MPU6050_IsDRY(void)
*Function:            Check interrupt pin of MPU6050
return 1:             Transition is done
       0:             Data register has not updated
*******************************************************************************/
unsigned char MPU6050_IsDRY(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_SET)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/******************************************************************************
*Function prototype:  void MPU6050_SetClockSource(uint8_t source)
*Function:            Set the clock source of MPU6050
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_SetClockSource(uint8_t source)
{
    IICWriteBits(DevAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
                          MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050_Reset(void)
{
    IICWriteBit(DevAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
    IICWriteBits(DevAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
                          MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/******************************************************************************
*Function prototype:  void MPU6050_SetFullScaleAccelRange(uint8_t range)
*Function:            Set the maximum range of MPU6050's accelerometer
*******************************************************************************/
void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
    IICWriteBits(DevAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
                          MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/******************************************************************************
*Function prototype:  void MPU6050_SetSleepEnabled(uint8_t enabled)
*Function:            Set MPU6050 whether to enter sleep mode
                      enabled = 1   sleep
                      enabled = 0   work
*******************************************************************************/
void MPU6050_SetSleepEnabled(uint8_t enabled)
{
    IICWriteBit(DevAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/******************************************************************************
*Function prototype:  uint8_t MPU6050_GetDeviceID(void)
*Function:            Read the WHO_AM_I flag of MPU6050 and return 0x68
*******************************************************************************/
uint8_t MPU6050_GetDeviceID(void)
{
    IICReadBytes(DevAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/******************************************************************************
*Function prototype:  uint8_t MPU6050_TestConnection(void)
*Function:            Test MPU6050 whether have connected
*******************************************************************************/
uint8_t MPU6050_TestConnection(void)
{
    if (MPU6050_GetDeviceID() == 0x68)  /* 0b01101000 */
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/******************************************************************************
*Function prototype:  void MPU6050_SetIICMasterModeEnabled(uint8_t enabled)
*Function:            Set MPU6050 whether is the master of AUX I2C
*******************************************************************************/
void MPU6050_SetIICMasterModeEnabled(uint8_t enabled)
{
    IICWriteBit(DevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/******************************************************************************
*Function prototype:  void MPU6050_SetIICBypassEnabled(uint8_t enabled)
*Function:
*******************************************************************************/
void MPU6050_SetIICBypassEnabled(uint8_t enabled)
{
    IICWriteBit(DevAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/******************************************************************************
*Function prototype:  void MPU6050_Check()
*Function:            Test the MPU6050 whether is on IIC bus
*******************************************************************************/
void MPU6050_Check(void)
{
    switch (MPU6050_TestConnection())
    {
        case 0: // printf("MPU6050 not found...\r\n");
        break;

        case 1: // printf("MPU6050 check success...\r\n");
        break;
    }
}

/******************************************************************************
*Function prototype:  void MPU6050_Init(void)
*Function:            Initialize MPU6050
*******************************************************************************/
void MPU6050_Init(void)
{
    IICWriteByte(DevAddr, MPU6050_RA_PWR_MGMT_1, 0x80);      /* PWR_MGMT_1  -- DEVICE_RESET 1 */
    // LED_SetLight(ON, ON, ON, ON);
    delay_ms(50);
    IICWriteByte(DevAddr, MPU6050_RA_SMPLRT_DIV, 0x00);      /* SMPLRT_DIV  -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) */
    IICWriteByte(DevAddr, MPU6050_RA_PWR_MGMT_1, 0x03);      /* PWR_MGMT_1  -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference) */
    IICWriteByte(DevAddr, MPU6050_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  /* INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS */
    IICWriteByte(DevAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);  /* CONFIG  -- EXT_SYNC_SET 0 (disable input pin for data sync); default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz) */
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    /* Accel scale 8g (4096 LSB/g) */
    IICWriteByte(DevAddr, MPU6050_RA_ACCEL_CONFIG, 2 << 3);
}

void MPU6050_ReadAcc(int16_t *accData)
{
    uint8_t buf[6];
    IICReadBytes(DevAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

void MPU6050_ReadGyro(int16_t *gyroData)
{
    uint8_t buf[6];
    IICReadBytes(DevAddr, MPU6050_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

/* Calibrate the offset of DMP */
void MPU6050_SetAccOffset(int16_t offset[3])
{
    uint8_t buf[2];
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        buf[0] = offset[i] >> 8;
        buf[1] = offset[i];
        IICWriteBytes(DevAddr, MPU6050_RA_XA_OFFS_H + i * 2, 2, buf);
    }
}

void MPU6050_SetGyroOffset(int16_t offset[3])
{
    uint8_t buf[2];
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        buf[0] = offset[i] >> 8;
        buf[1] = offset[i];
        IICWriteBytes(DevAddr, MPU6050_RA_XG_OFFS_USRH + i * 2, 2, buf);
    }
}

/* BANK_SEL register */
void MPU6050_SetMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank)
{
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    IICWriteByte(DevAddr, MPU6050_RA_BANK_SEL, bank);
}

/* MEM_START_ADDR register */
void MPU6050_SetMemoryStartAddress(uint8_t address)
{
    IICWriteByte(DevAddr, MPU6050_RA_MEM_START_ADDR, address);
}

/* MEM_R_W register */
uint8_t MPU6050_ReadMemoryByte(void)
{
    IICReadBytes(DevAddr, MPU6050_RA_MEM_R_W, 1, buffer);
    return buffer[0];
}

/* XG_OFFS_USR* registers */
int16_t MPU6050_GetXGyroOffset(void)
{
    IICReadBytes(DevAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t MPU6050_GetYGyroOffset(void)
{
    IICReadBytes(DevAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t MPU6050_GetZGyroOffset(void)
{
    IICReadBytes(DevAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

uint8_t verifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
uint8_t progBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];

uint8_t MPU6050_WriteMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem)
{
    uint8_t chunkSize;
    //uint8_t *verifyBuffer;
    uint8_t *tprogBuffer;
    uint16_t i;
    uint8_t j;
    MPU6050_SetMemoryBank(bank, 0, 0);
    MPU6050_SetMemoryStartAddress(address);

    for (i = 0; i < dataSize;)
    {
        /* Determine correct chunk size according to bank position and data size */
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        /* Make sure we don't go past the data size */
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        /* Make sure this chunk doesn't go past the bank boundary (256 bytes) */
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        /* Write the chunk of data as specified */
        tprogBuffer = (uint8_t*)data + i;
        IICWriteBytes(DevAddr, MPU6050_RA_MEM_R_W, chunkSize, tprogBuffer);

        /* Verify data if needed */
        if (verify)
        {
            MPU6050_SetMemoryBank(bank, 0, 0);
            MPU6050_SetMemoryStartAddress(address);
            IICReadBytes(DevAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);

            for (j = 0; j < chunkSize; j++)
            {
                if (tprogBuffer[j] != verifyBuffer[j])
                {
                    return 0;
                }
            }
        }

        /* Increase byte index by [chunkSize] */
        i += chunkSize;

        /* uint8_tautomatically wraps to 0 at 256 */
        address += chunkSize;

        /* If we aren't done, update bank (if necessary) and address */
        if (i < dataSize)
        {
            if (address == 0) bank++;
            MPU6050_SetMemoryBank(bank, 0, 0);
            MPU6050_SetMemoryStartAddress(address);
        }
    }
    return 1;
}

uint8_t MPU6050_WriteDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, uint8_t useProgMem)
{
    uint8_t *progBuffer, success, special;
    uint16_t i;

    /* Config set data is a long string of blocks with the following structure: */
    /* [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]] */
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;)
    {
        bank = data[i++];
        offset = data[i++];
        length = data[i++];

        /* Write data or perform special action */
        if (length > 0)
        {
            /* Regular block of data to write */
            progBuffer = (uint8_t*)data + i;
            success = MPU6050_WriteMemoryBlock(progBuffer, length, bank, offset, 1, 0);
            i += length;
        }
        else
        {
            /* Special instruction */
            /* NOTE: this kind of behavior (what and when to do certain things) */
            /* is totally undocumented. This code is in here based on observed */
            /* behavior only, and exactly why (or even whether) it has to be here */
            /* is anybody's guess for now. */
            special = data[i++];
            if (special == 0x01)
            {
                /* Enable DMP-related interrupts */
                IICWriteByte(DevAddr, MPU6050_RA_INT_ENABLE, 0x32);  /* Single operation */
                success = 1;
            }
            else
            {
                /* Unknown special command */
                success = 0;
            }
        }

        if (!success)
        {
            return 0;
        }
    }
    return 1;
}

uint8_t MPU6050_WriteProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify)
{
    return MPU6050_WriteMemoryBlock(data, dataSize, bank, address, verify, 0);
}

uint8_t MPU6050_WriteProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize)
{
    return MPU6050_WriteDMPConfigurationSet(data, dataSize, 0);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void MPU6050_SetIntEnabled(uint8_t enabled)
{
    IICWriteByte(DevAddr, MPU6050_RA_INT_ENABLE, enabled);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void MPU6050_SetRate(uint8_t rate)
{
    IICWriteByte(DevAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void MPU6050_SetDLPFMode(uint8_t mode)
{
    IICWriteBits(DevAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT,
                          MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void MPU6050_SetExternalFrameSync(uint8_t sync)
{
    IICWriteBits(DevAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
                          MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

void MPU6050_SetDMPConfig1(uint8_t config)
{
    IICWriteByte(DevAddr, MPU6050_RA_DMP_CFG_1, config);
}

void MPU6050_SetDMPConfig2(uint8_t config)
{
    IICWriteByte(DevAddr, MPU6050_RA_DMP_CFG_2, config);
}

void MPU6050_SetOTPBankValid(uint8_t enabled)
{
    IICWriteBit(DevAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

void MPU6050_SetXGyroOffset(int16_t offset)
{
    buffer[0] = offset >> 8;
    buffer[1] = offset & 0x00ff;
    IICWriteBytes(DevAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
}

void MPU6050_SetYGyroOffset(int16_t offset)
{
    buffer[0] = offset >> 8;
    buffer[1] = offset & 0x00ff;
    IICWriteBytes(DevAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
}

void MPU6050_SetZGyroOffset(int16_t offset)
{
    buffer[0] = offset >> 8;
    buffer[1] = offset & 0x00ff;
    IICWriteBytes(DevAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050_ResetFIFO(void)
{
    IICWriteBit(DevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU6050_GetFIFOCount(void)
{
    IICReadBytes(DevAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void MPU6050_SetMotionDetectionThreshold(uint8_t threshold)
{
    IICWriteByte(DevAddr, MPU6050_RA_MOT_THR, threshold);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void MPU6050_SetZeroMotionDetectionThreshold(uint8_t threshold)
{
    IICWriteByte(DevAddr, MPU6050_RA_ZRMOT_THR, threshold);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void MPU6050_SetMotionDetectionDuration(uint8_t duration)
{
    IICWriteByte(DevAddr, MPU6050_RA_MOT_DUR, duration);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void MPU6050_SetZeroMotionDetectionDuration(uint8_t duration)
{
    IICWriteByte(DevAddr, MPU6050_RA_ZRMOT_DUR, duration);
}

void MPU6050_ReadMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
    uint8_t chunkSize;
    uint16_t i;
    MPU6050_SetMemoryBank(bank, 0, 0);
    MPU6050_SetMemoryStartAddress(address);

    for (i = 0; i < dataSize;)
    {
        /* Determine correct chunk size according to bank position and data size */
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        /* Make sure we don't go past the data size */
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        /* Make sure this chunk doesn't go past the bank boundary (256 bytes) */
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        /* Read the chunk of data as specified */
        IICWriteBytes(DevAddr, MPU6050_RA_MEM_R_W, chunkSize, data + i);

        /* Increase byte index by [chunkSize] */
        i += chunkSize;

        /* uint8_tautomatically wraps to 0 at 256 */
        address += chunkSize;

        /* If we aren't done, update bank (if necessary) and address */
        if (i < dataSize)
        {
            if (address == 0) bank++;
            MPU6050_SetMemoryBank(bank, 0, 0);
            MPU6050_SetMemoryStartAddress(address);
        }
    }
}

void MPU6050_GetFIFOBytes(uint8_t *data, uint8_t length)
{
    IICReadBytes(DevAddr, MPU6050_RA_FIFO_R_W, length, data);
}

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPU6050_GetIntStatus(void)
{
    return IIC_ReadOneByte(DevAddr, MPU6050_RA_INT_STATUS);
}

void MPU6050_SetDMPEnabled(uint8_t enabled)
{
    IICWriteBit(DevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

uint8_t MPU6050_GetOTPBankValid(void)
{
    uint8_t temp = IIC_ReadOneByte(DevAddr, MPU6050_RA_XG_OFFS_TC);
    return temp & (1 << MPU6050_TC_OTP_BNK_VLD_BIT);
}

int8_t MPU6050_GetXGyroOffsetTC(void)
{
    uint8_t temp = IIC_ReadOneByte(DevAddr, MPU6050_RA_XG_OFFS_TC);
    temp &= 0x3F;
    return temp;
}

void MPU6050_SetXGyroOffsetTC(int8_t offset)
{
    IICWriteBits(DevAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/* YG_OFFS_TC register */
int8_t MPU6050_GetYGyroOffsetTC(void)
{
    uint8_t temp = IIC_ReadOneByte(DevAddr, MPU6050_RA_YG_OFFS_TC);
    temp &= 0x3F;
    return temp;
}

void MPU6050_SetYGyroOffsetTC(int8_t offset)
{
    IICWriteBits(DevAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/* ZG_OFFS_TC register */
int8_t MPU6050_GetZGyroOffsetTC(void)
{
    uint8_t temp = IIC_ReadOneByte(DevAddr, MPU6050_RA_ZG_OFFS_TC);
    temp &= 0x3F;
    return temp;
}

void MPU6050_SetZGyroOffsetTC(int8_t offset)
{
    IICWriteBits(DevAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void MPU6050_SetSlaveAddress(uint8_t num, uint8_t address)
{
    if (num > 3) return;
    IICWriteByte(DevAddr, MPU6050_RA_I2C_SLV0_ADDR + num * 3, address);
}

/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void MPU6050_ResetIICMaster(void)
{
    IICWriteBit(DevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, 1);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPU6050_SetFIFOEnabled(uint8_t enabled)
{
    IICWriteBit(DevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

void MPU6050_ResetDMP(void)
{
    IICWriteBit(DevAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}
