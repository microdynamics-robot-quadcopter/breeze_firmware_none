/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_mpu6050.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.04
Description: Implement the mpu6050 function
Others:      none
Function List:
             1.  void MPU6050_GetFIFOBytes(u8 *data, u8 length);
             2.  void MPU6050_Init(void);
             3.  void MPU6050_ReadAcc(s16 *data);
             4.  void MPU6050_ReadGyr(s16 *data);
             5.  void MPU6050_ReadMemoryBlock(u8 *data, u16 data_size, u8 bank,
                                              u8 address);
             6.  void MPU6050_Reset(void);
             7.  void MPU6050_ResetDMP(void);
             8.  void MPU6050_ResetFIFO(void);
             9.  void MPU6050_ResetIICMaster(void);
             10. void MPU6050_SetAccOffset(s16 offset[3]);
             11. void MPU6050_SetClockSource(u8 source);
             12. void MPU6050_SetDLPFMode(u8 mode);
             13. void MPU6050_SetDMPConfig1(u8 config);
             14. void MPU6050_SetDMPConfig2(u8 config);
             15. void MPU6050_SetDMPEnabled(u8 enabled);
             16. void MPU6050_SetExternalFrameSync(u8 sync);
             17. void MPU6050_SetSleepEnabled(u8 enabled);
             18. void MPU6050_SetMemoryBank(u8 bank, u8 prefetch_enable,
                                            u8 user_bank);
             19. void MPU6050_SetMemoryStartAddress(u8 address);
             20. void MPU6050_SetIntEnabled(u8 enabled);
             21. void MPU6050_SetRate(u8 rate);
             22. void MPU6050_SetFIFOEnabled(u8 enabled);
             23. void MPU6050_SetFullScaleAccRange(u8 range);
             24. void MPU6050_SetFullScaleGyrRange(u8 range);
             25. void MPU6050_SetGyrOffset(s16 offset[3]);
             26. void MPU6050_SetIICBypassEnabled(u8 enabled);
             27. void MPU6050_SetIICMasterModeEnabled(u8 enabled);
             28. void MPU6050_SetMotionDetectionDuration(u8 duration);
             29. void MPU6050_SetMotionDetectionThreshold(u8 threshold);
             30. void MPU6050_SetOTPBankValid(u8 enabled);
             31. void MPU6050_SetSlaveAddress(u8 numbers, u8 address);
             32. void MPU6050_SetXGyrOffset(s16 offset);
             33. void MPU6050_SetXGyrOffsetTC(s8 offset);
             34. void MPU6050_SetYGyrOffset(s16 offset);
             35. void MPU6050_SetYGyrOffsetTC(s8 offset);
             36. void MPU6050_SetZeroMotionDetectionDuration(u8 duration);
             37. void MPU6050_SetZeroMotionDetectionThreshold(u8 threshold);
             38. void MPU6050_SetZGyrOffset(s16 offset);
             39. void MPU6050_SetZGyrOffsetTC(s8 offset);
             40. u8   MPU6050_Check(void);
             41. u8   MPU6050_GetDeviceID(void);
             42. u8   MPU6050_GetIntStatus(void);
             43. u8   MPU6050_GetOTPBankValid(void);
             44. u8   MPU6050_IsDRY(void);
             45. u8   MPU6050_ReadMemoryByte(void);
             46. u8   MPU6050_TestConnection(void);
             47. u8   MPU6050_WriteDMPConfigurationSet(uc8 *data,
                                                       u16 data_size);
             48. u8   MPU6050_WriteMemoryBlock(uc8 *data, u16 data_size,
                                               u8 bank, u8 address,
                                               u8 verify_flag);
             49. u8   MPU6050_WriteProgMemoryBlock(uc8 *data, u16 data_size,
                                                   u8 bank, u8 address,
                                                   u8 verify_flag);
             50. u8   MPU6050_WriteProgDMPConfigurationSet(uc8 *data,
                                                           u16 data_size);
             51. s8   MPU6050_GetXGyrOffsetTC(void);
             52. s8   MPU6050_GetYGyrOffsetTC(void);
             53. s8   MPU6050_GetZGyrOffsetTC(void);
             54. u16  MPU6050_GetFIFOCount(void);
             55. s16  MPU6050_GetXGyrOffset(void);
             56. s16  MPU6050_GetYGyrOffset(void);
             57. s16  MPU6050_GetZGyrOffset(void);
History:
maksyuki    2017.01.11    Modify the module
myyerrol    2017.04.22    Format the module
*******************************************************************************/

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_iic.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_mpu6050.h"

static u8 mpu6050_buffer[14];
static u8 verify_buffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];

void MPU6050_GetFIFOBytes(u8 *data, u8 length)
{
    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_FIFO_R_W, length, data);
}

void MPU6050_Init(void)
{
    // PWR_MGMT_1: DEVICE_RESET 1.
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_PWR_MGMT_1, 0X80);
    Delay_TimeMs(50);
    // SMPLRT_DIV: SMPLRT_DIV = 0.
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV).
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_SMPLRT_DIV, 0X00);
    // PWR_MGMT_1: SLEEP 0, CYCLE 0, TEMP_DIS 0,
    // CLKSEL 3(PLL with Z Gyr reference).
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_PWR_MGMT_1, 0X03);
    // INT_PIN_CFG: INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS,
    // INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN,
    // CLOCK_DIS.
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_INT_PIN_CFG,
                  0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 |
                  1 << 1 | 0 << 0);
    // CONFIG: EXT_SYNC_SET 0 (disable input pin for data sync), default
    // DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz).
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);
    MPU6050_SetFullScaleGyrRange(MPU6050_GYRO_FS_2000);
    // Acceleration scale 8g (4096 LSB/g).
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_ACCEL_CONFIG, 2 << 3);
}

void MPU6050_ReadAcc(s16 *data)
{
    u8 temp[6];

    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_ACCEL_XOUT_H, 6, temp);

    data[0] = (s16)((temp[0] << 8) | temp[1]);
    data[1] = (s16)((temp[2] << 8) | temp[3]);
    data[2] = (s16)((temp[4] << 8) | temp[5]);
}

void MPU6050_ReadGyr(s16 *data)
{
    u8 temp[6];

    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_GYRO_XOUT_H, 6, temp);

    data[0] = (s16)((temp[0] << 8) | temp[1]);
    data[1] = (s16)((temp[2] << 8) | temp[3]);
    data[2] = (s16)((temp[4] << 8) | temp[5]);
}

void MPU6050_ReadMemoryBlock(u8 *data, u16 data_size, u8 bank, u8 address)
{
    u8  chunk_size;
    u16 i;

    MPU6050_SetMemoryBank(bank, 0, 0);
    MPU6050_SetMemoryStartAddress(address);

    for (i = 0; i < data_size;)
    {
        // Determine correct chunk size according to bank position and data
        // size.
        chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;
        // Make sure we don't go past the data size.
        if (i + chunk_size > data_size)
        {
            chunk_size = data_size - i;
        }
        // Make sure this chunk doesn't go past the bank boundary(256 bytes).
        if (chunk_size > 256 - address)
        {
            chunk_size = 256 - address;
        }
        // Read the chunk of data as specified.
        IIC_WriteBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_MEM_R_W, chunk_size,
                       data + i);
        // Increase byte index by [chunk_size].
        i += chunk_size;
        // uint8_t automatically wraps to 0 at 256.
        address += chunk_size;
        // If we aren't done, update bank and address.
        if (i < data_size)
        {
            if (address == 0)
            {
                bank++;
            }
            MPU6050_SetMemoryBank(bank, 0, 0);
            MPU6050_SetMemoryStartAddress(address);
        }
    }
}

// Trigger a full device reset. A small delay of ~50ms may be desirable after
// triggering a reset.
void MPU6050_Reset(void)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_PWR_MGMT_1,
                 MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

void MPU6050_ResetDMP(void)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_USER_CTRL,
                 MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}

// Reset the FIFO. This bit resets the FIFO mpu6050_buffer when set to 1 while
// FIFO_EN equals 0. This bit automatically clears to 0 after the reset has
// been triggered.
void MPU6050_ResetFIFO(void)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_USER_CTRL,
                 MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

// Reset the I2C Master. This bit resets the I2C Master when set to 1 while
// I2C_MST_EN equals 0. This bit automatically clears to 0 after the reset has
// been triggered.
void MPU6050_ResetIICMaster(void)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_USER_CTRL,
                 MPU6050_USERCTRL_I2C_MST_RESET_BIT, 1);
}

// Calibrate the offset of DMP.
void MPU6050_SetAccOffset(s16 offset[3])
{
    u8 i;
    u8 temp[2];

    for (i = 0; i < 3; i++)
    {
        temp[0] = offset[i] >> 8;
        temp[1] = offset[i];
        IIC_WriteBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_XA_OFFS_H + i * 2, 2,
                       temp);
    }
}

// Set the clock source of MPU6050.
// CLK_SEL | Clock Source
// --------+--------------------------------------
// 0       | Internal oscillator
// 1       | PLL with X Gyr reference
// 2       | PLL with Y Gyr reference
// 3       | PLL with Z Gyr reference
// 4       | PLL with external 32.768kHz reference
// 5       | PLL with external 19.2MHz reference
// 6       | Reserved
// 7       | Stops the clock and keeps the timing generator in reset
void MPU6050_SetClockSource(u8 source)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_PWR_MGMT_1,
                  MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

// Set digital low-pass filter configuration.
void MPU6050_SetDLPFMode(u8 mode)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_CONFIG,
                  MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050_SetDMPConfig1(u8 config)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_DMP_CFG_1, config);
}

void MPU6050_SetDMPConfig2(u8 config)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_DMP_CFG_2, config);
}

void MPU6050_SetDMPEnabled(u8 enabled)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_USER_CTRL,
                 MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

// Set external FSYNC configuration.
void MPU6050_SetExternalFrameSync(u8 sync)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_CONFIG,
                  MPU6050_CFG_EXT_SYNC_SET_BIT,
                  MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

// Set MPU6050 whether to enter sleep mode.
// enable = 1: sleep.
// enable = 0: work.
void MPU6050_SetSleepEnabled(u8 enabled)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_PWR_MGMT_1,
                 MPU6050_PWR1_SLEEP_BIT, enabled);
}

void MPU6050_SetMemoryBank(u8 bank, u8 prefetch_enable, u8 user_bank)
{
    bank &= 0X1F;

    if (user_bank)
    {
        bank |= 0X20;
    }
    if (prefetch_enable)
    {
        bank |= 0X40;
    }

    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_BANK_SEL, bank);
}

void MPU6050_SetMemoryStartAddress(u8 address)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_MEM_START_ADDR, address);
}

// Set full interrupt enabled status. Full register byte for all interrupts,
// for quick reading. Each bit should be set 0 for disabled, 1 for enabled.
void MPU6050_SetIntEnabled(u8 enabled)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_INT_ENABLE, enabled);
}

// Set gyroscope sample rate divider.
void MPU6050_SetRate(u8 rate)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_SMPLRT_DIV, rate);
}

// Set FIFO enabled status.
void MPU6050_SetFIFOEnabled(u8 enabled)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_USER_CTRL,
                 MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

// Set the maximum range of MPU6050's accelerometer
void MPU6050_SetFullScaleAccRange(u8 range)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_ACCEL_CONFIG,
                  MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH,
                  range);
}

// Set full-scale gyroscope range.
void MPU6050_SetFullScaleGyrRange(u8 range)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_GYRO_CONFIG,
                  MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH,
                  range);
}

void MPU6050_SetGyrOffset(s16 offset[3])
{
    u8 i;
    u8 temp[2];

    for (i = 0; i < 3; i++)
    {
        temp[0] = offset[i] >> 8;
        temp[1] = offset[i];
        IIC_WriteBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_XG_OFFS_USRH + i * 2, 2,
                       temp);
    }
}

void MPU6050_SetIICBypassEnabled(u8 enabled)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_INT_PIN_CFG,
                 MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

// Set MPU6050 whether is the master of AUX I2C.
void MPU6050_SetIICMasterModeEnabled(u8 enabled)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_USER_CTRL,
                 MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

// Set motion detection event duration threshold.
void MPU6050_SetMotionDetectionDuration(u8 duration)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_MOT_DUR, duration);
}

// Set free-fall event acceleration threshold.
void MPU6050_SetMotionDetectionThreshold(u8 threshold)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_MOT_THR, threshold);
}

void MPU6050_SetOTPBankValid(u8 enabled)
{
    IIC_WriteBit(MPU6050_DEVICE_ADDR, MPU6050_RA_XG_OFFS_TC,
                 MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

// Set the I2C address of the specified slave (0-3).
void MPU6050_SetSlaveAddress(u8 numbers, u8 address)
{
    if (numbers > 3)
    {
        return;
    }

    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_I2C_SLV0_ADDR + numbers * 3,
                  address);
}

void MPU6050_SetXGyrOffset(s16 offset)
{
    mpu6050_buffer[0] = offset >> 8;
    mpu6050_buffer[1] = offset & 0X00FF;

    IIC_WriteBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_XG_OFFS_USRH, 2,
                   mpu6050_buffer);
}

void MPU6050_SetXGyrOffsetTC(s8 offset)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_XG_OFFS_TC,
                  MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

void MPU6050_SetYGyrOffset(s16 offset)
{
    mpu6050_buffer[0] = offset >> 8;
    mpu6050_buffer[1] = offset & 0X00FF;

    IIC_WriteBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_YG_OFFS_USRH, 2,
                   mpu6050_buffer);
}

void MPU6050_SetYGyrOffsetTC(s8 offset)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_YG_OFFS_TC,
                  MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// Set zero motion detection event duration threshold.
void MPU6050_SetZeroMotionDetectionDuration(u8 duration)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_ZRMOT_DUR, duration);
}

// Set zero motion detection event acceleration threshold.
void MPU6050_SetZeroMotionDetectionThreshold(u8 threshold)
{
    IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_ZRMOT_THR, threshold);
}

void MPU6050_SetZGyrOffset(s16 offset)
{
    mpu6050_buffer[0] = offset >> 8;
    mpu6050_buffer[1] = offset & 0X00FF;

    IIC_WriteBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_ZG_OFFS_USRH, 2,
                   mpu6050_buffer);
}

void MPU6050_SetZGyrOffsetTC(s8 offset)
{
    IIC_WriteBits(MPU6050_DEVICE_ADDR, MPU6050_RA_ZG_OFFS_TC,
                  MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

u8 MPU6050_Check(void)
{
    if (MPU6050_TestConnection())
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// Read the WHO_AM_I flag of MPU6050 and return 0X68.
u8 MPU6050_GetDeviceID(void)
{
    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_WHO_AM_I, 1, mpu6050_buffer);
    return mpu6050_buffer[0];
}

// Get full set of interrupt status bits. These bits clear to 0 after the
// register has been read. Very useful for getting multiple INT statuses, since
// each single bit read clears all of them because it has to read the whole
// byte.
u8 MPU6050_GetIntStatus(void)
{
    return IIC_ReadByte(MPU6050_DEVICE_ADDR, MPU6050_RA_INT_STATUS);
}

u8 MPU6050_GetOTPBankValid(void)
{
    u8 temp = IIC_ReadByte(MPU6050_DEVICE_ADDR, MPU6050_RA_XG_OFFS_TC);
    return temp & (1 << MPU6050_TC_OTP_BNK_VLD_BIT);
}

// Check interrupt pin of MPU6050.
// return 1: Transition is done.
// return 0: Data register has not updated.
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

u8 MPU6050_ReadMemoryByte(void)
{
    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_MEM_R_W, 1, mpu6050_buffer);
    return mpu6050_buffer[0];
}

// Test MPU6050 whether have connected.
u8 MPU6050_TestConnection(void)
{
    if (MPU6050_GetDeviceID() == 0X68)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

u8 MPU6050_WriteDMPConfigurationSet(uc8 *data, u16 data_size)
{
    u8  success;
    u8  special;
    u8 *prog_buffer;
    u16 i;

    // Config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]].
    u8 bank, offset, length;

    for (i = 0; i < data_size;)
    {
        bank   = data[i++];
        offset = data[i++];
        length = data[i++];

        // Write data or perform special action.
        if (length > 0)
        {
            // Write regular blocks of data.
            prog_buffer = (u8 *)data + i;
            success     = MPU6050_WriteMemoryBlock(prog_buffer, length, bank,
                                                   offset, 1);
            i          += length;
        }
        else
        {
            // Special instruction.
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be
            // here is anybody's guess for now.
            special = data[i++];
            if (special == 0X01)
            {
                // Enable DMP-related interrupts.
                // Single operation.
                IIC_WriteByte(MPU6050_DEVICE_ADDR, MPU6050_RA_INT_ENABLE,
                              0X32);
                success = 1;
            }
            else
            {
                // Unknown special command.
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

u8 MPU6050_WriteMemoryBlock(uc8 *data, u16 data_size, u8 bank, u8 address,
                            u8 verify_flag)
{
    u8  j;
    u8  chunk_size;
    u8 *prog_buffer;
    u16 i;

    MPU6050_SetMemoryBank(bank, 0, 0);
    MPU6050_SetMemoryStartAddress(address);

    for (i = 0; i < data_size;)
    {
        // Determine correct chunk size according to bank position and data
        // size.
        chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;
        // Make sure we don't go past the data size.
        if (i + chunk_size > data_size)
        {
            chunk_size = data_size - i;
        }
        // Make sure this chunk doesn't go past the bank boundary(256 bytes).
        if (chunk_size > 256 - address)
        {
            chunk_size = 256 - address;
        }
        // Write the chunk of data as specified.
        prog_buffer = (u8 *)data + i;
        IIC_WriteBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_MEM_R_W, chunk_size,
                       prog_buffer);
        // Verify data if needed.
        if (verify_flag)
        {
            MPU6050_SetMemoryBank(bank, 0, 0);
            MPU6050_SetMemoryStartAddress(address);
            IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_MEM_R_W, chunk_size,
                          verify_buffer);
            for (j = 0; j < chunk_size; j++)
            {
                if (prog_buffer[j] != verify_buffer[j])
                {
                    return 0;
                }
            }
        }
        // Increase byte index by [chunk_size].
        i += chunk_size;
        // uint8_t automatically wraps to 0 at 256.
        address += chunk_size;
        // If we aren't done, update bank and address.
        if (i < data_size)
        {
            if (address == 0)
            {
                bank++;
            }
            MPU6050_SetMemoryBank(bank, 0, 0);
            MPU6050_SetMemoryStartAddress(address);
        }
    }

    return 1;
}

u8 MPU6050_WriteProgMemoryBlock(uc8 *data, u16 data_size, u8 bank, u8 address,
                                u8 verify)
{
    return MPU6050_WriteMemoryBlock(data, data_size, bank, address, verify);
}

u8 MPU6050_WriteProgDMPConfigurationSet(uc8 *data, u16 data_size)
{
    return MPU6050_WriteDMPConfigurationSet(data, data_size);
}

s8 MPU6050_GetXGyrOffsetTC(void)
{
    u8 temp = IIC_ReadByte(MPU6050_DEVICE_ADDR, MPU6050_RA_XG_OFFS_TC);
    temp   &= 0X3F;
    return temp;
}

s8 MPU6050_GetYGyrOffsetTC(void)
{
    u8 temp = IIC_ReadByte(MPU6050_DEVICE_ADDR, MPU6050_RA_YG_OFFS_TC);
    temp   &= 0X3F;
    return temp;
}

s8 MPU6050_GetZGyrOffsetTC(void)
{
    u8 temp = IIC_ReadByte(MPU6050_DEVICE_ADDR, MPU6050_RA_ZG_OFFS_TC);
    temp   &= 0X3F;
    return temp;
}

// Get current FIFO mpu6050_buffer size. This value indicates the number of
// bytes stored in the FIFO mpu6050_buffer. This number is in turn the number
// of bytes that can be read from the FIFO mpu6050_buffer and it is directly
// proportional to the number of samples available given the set of sensor data
// bound to be stored in the FIFO(register 35 and 36).
u16 MPU6050_GetFIFOCount(void)
{
    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_FIFO_COUNTH, 2,
                  mpu6050_buffer);
    return (((u16)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1];
}

s16 MPU6050_GetXGyrOffset(void)
{
    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_XG_OFFS_USRH, 2,
                  mpu6050_buffer);
    return (((s16)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1];
}

s16 MPU6050_GetYGyrOffset(void)
{
    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_YG_OFFS_USRH, 2,
                  mpu6050_buffer);
    return (((s16)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1];
}

s16 MPU6050_GetZGyrOffset(void)
{
    IIC_ReadBytes(MPU6050_DEVICE_ADDR, MPU6050_RA_ZG_OFFS_USRH, 2,
                  mpu6050_buffer);
    return (((s16)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1];
}
