/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_mpu6050.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.09.04
Description: Declare the mpu6050 function
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
<author>    <date>        <desc>
maksyuki    2017.01.11    Modify the module
myyerrol    2017.04.22    Format the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_MPU6050_H__
#define __STM32F10X_MODULE_MPU6050_H__

#include "stm32f10x.h"

#define MPU6050_DEVICE_ADDR                    0XD0

// [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD.
#define MPU6050_RA_XG_OFFS_TC                  0X00
// [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD.
#define MPU6050_RA_YG_OFFS_TC                  0X01
// [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD.
#define MPU6050_RA_ZG_OFFS_TC                  0X02
// [7:0] X_FINE_GAIN.
#define MPU6050_RA_X_FINE_GAIN                 0X03
// [7:0] Y_FINE_GAIN.
#define MPU6050_RA_Y_FINE_GAIN                 0X04
// [7:0] Z_FINE_GAIN.
#define MPU6050_RA_Z_FINE_GAIN                 0X05
// [15:0] XA_OFFS.
#define MPU6050_RA_XA_OFFS_H                   0X06
#define MPU6050_RA_XA_OFFS_L_TC                0X07
// [15:0] YA_OFFS.
#define MPU6050_RA_YA_OFFS_H                   0X08
#define MPU6050_RA_YA_OFFS_L_TC                0X09
// [15:0] ZA_OFFS.
#define MPU6050_RA_ZA_OFFS_H                   0X0A
#define MPU6050_RA_ZA_OFFS_L_TC                0X0B
// [15:0] XG_OFFS_USR.
#define MPU6050_RA_XG_OFFS_USRH                0X13
#define MPU6050_RA_XG_OFFS_USRL                0X14
// [15:0] YG_OFFS_USR.
#define MPU6050_RA_YG_OFFS_USRH                0X15
#define MPU6050_RA_YG_OFFS_USRL                0X16
// [15:0] ZG_OFFS_USR.
#define MPU6050_RA_ZG_OFFS_USRH                0X17
#define MPU6050_RA_ZG_OFFS_USRL                0X18
#define MPU6050_RA_SMPLRT_DIV                  0X19
#define MPU6050_RA_CONFIG                      0X1A
#define MPU6050_RA_GYRO_CONFIG                 0X1B
#define MPU6050_RA_ACCEL_CONFIG                0X1C
#define MPU6050_RA_FF_THR                      0X1D
#define MPU6050_RA_FF_DUR                      0X1E
#define MPU6050_RA_MOT_THR                     0X1F
#define MPU6050_RA_MOT_DUR                     0X20
#define MPU6050_RA_ZRMOT_THR                   0X21
#define MPU6050_RA_ZRMOT_DUR                   0X22
#define MPU6050_RA_FIFO_EN                     0X23
#define MPU6050_RA_I2C_MST_CTRL                0X24
#define MPU6050_RA_I2C_SLV0_ADDR               0X25
#define MPU6050_RA_I2C_SLV0_REG                0X26
#define MPU6050_RA_I2C_SLV0_CTRL               0X27
#define MPU6050_RA_I2C_SLV1_ADDR               0X28
#define MPU6050_RA_I2C_SLV1_REG                0X29
#define MPU6050_RA_I2C_SLV1_CTRL               0X2A
#define MPU6050_RA_I2C_SLV2_ADDR               0X2B
#define MPU6050_RA_I2C_SLV2_REG                0X2C
#define MPU6050_RA_I2C_SLV2_CTRL               0X2D
#define MPU6050_RA_I2C_SLV3_ADDR               0X2E
#define MPU6050_RA_I2C_SLV3_REG                0X2F
#define MPU6050_RA_I2C_SLV3_CTRL               0X30
#define MPU6050_RA_I2C_SLV4_ADDR               0X31
#define MPU6050_RA_I2C_SLV4_REG                0X32
#define MPU6050_RA_I2C_SLV4_DO                 0X33
#define MPU6050_RA_I2C_SLV4_CTRL               0X34
#define MPU6050_RA_I2C_SLV4_DI                 0X35
#define MPU6050_RA_I2C_MST_STATUS              0X36
#define MPU6050_RA_INT_PIN_CFG                 0X37
#define MPU6050_RA_INT_ENABLE                  0X38
#define MPU6050_RA_DMP_INT_STATUS              0X39
#define MPU6050_RA_INT_STATUS                  0X3A
#define MPU6050_RA_ACCEL_XOUT_H                0X3B
#define MPU6050_RA_ACCEL_XOUT_L                0X3C
#define MPU6050_RA_ACCEL_YOUT_H                0X3D
#define MPU6050_RA_ACCEL_YOUT_L                0X3E
#define MPU6050_RA_ACCEL_ZOUT_H                0X3F
#define MPU6050_RA_ACCEL_ZOUT_L                0X40
#define MPU6050_RA_TEMP_OUT_H                  0X41
#define MPU6050_RA_TEMP_OUT_L                  0X42
#define MPU6050_RA_GYRO_XOUT_H                 0X43
#define MPU6050_RA_GYRO_XOUT_L                 0X44
#define MPU6050_RA_GYRO_YOUT_H                 0X45
#define MPU6050_RA_GYRO_YOUT_L                 0X46
#define MPU6050_RA_GYRO_ZOUT_H                 0X47
#define MPU6050_RA_GYRO_ZOUT_L                 0X48
#define MPU6050_RA_EXT_SENS_DATA_00            0X49
#define MPU6050_RA_EXT_SENS_DATA_01            0X4A
#define MPU6050_RA_EXT_SENS_DATA_02            0X4B
#define MPU6050_RA_EXT_SENS_DATA_03            0X4C
#define MPU6050_RA_EXT_SENS_DATA_04            0X4D
#define MPU6050_RA_EXT_SENS_DATA_05            0X4E
#define MPU6050_RA_EXT_SENS_DATA_06            0X4F
#define MPU6050_RA_EXT_SENS_DATA_07            0X50
#define MPU6050_RA_EXT_SENS_DATA_08            0X51
#define MPU6050_RA_EXT_SENS_DATA_09            0X52
#define MPU6050_RA_EXT_SENS_DATA_10            0X53
#define MPU6050_RA_EXT_SENS_DATA_11            0X54
#define MPU6050_RA_EXT_SENS_DATA_12            0X55
#define MPU6050_RA_EXT_SENS_DATA_13            0X56
#define MPU6050_RA_EXT_SENS_DATA_14            0X57
#define MPU6050_RA_EXT_SENS_DATA_15            0X58
#define MPU6050_RA_EXT_SENS_DATA_16            0X59
#define MPU6050_RA_EXT_SENS_DATA_17            0X5A
#define MPU6050_RA_EXT_SENS_DATA_18            0X5B
#define MPU6050_RA_EXT_SENS_DATA_19            0X5C
#define MPU6050_RA_EXT_SENS_DATA_20            0X5D
#define MPU6050_RA_EXT_SENS_DATA_21            0X5E
#define MPU6050_RA_EXT_SENS_DATA_22            0X5F
#define MPU6050_RA_EXT_SENS_DATA_23            0X60
#define MPU6050_RA_MOT_DETECT_STATUS           0X61
#define MPU6050_RA_I2C_SLV0_DO                 0X63
#define MPU6050_RA_I2C_SLV1_DO                 0X64
#define MPU6050_RA_I2C_SLV2_DO                 0X65
#define MPU6050_RA_I2C_SLV3_DO                 0X66
#define MPU6050_RA_I2C_MST_DELAY_CTRL          0X67
#define MPU6050_RA_SIGNAL_PATH_RESET           0X68
#define MPU6050_RA_MOT_DETECT_CTRL             0X69
#define MPU6050_RA_USER_CTRL                   0X6A
#define MPU6050_RA_PWR_MGMT_1                  0X6B
#define MPU6050_RA_PWR_MGMT_2                  0X6C
#define MPU6050_RA_BANK_SEL                    0X6D
#define MPU6050_RA_MEM_START_ADDR              0X6E
#define MPU6050_RA_MEM_R_W                     0X6F
#define MPU6050_RA_DMP_CFG_1                   0X70
#define MPU6050_RA_DMP_CFG_2                   0X71
#define MPU6050_RA_FIFO_COUNTH                 0X72
#define MPU6050_RA_FIFO_COUNTL                 0X73
#define MPU6050_RA_FIFO_R_W                    0X74
#define MPU6050_RA_WHO_AM_I                    0X75

#define MPU6050_TC_PWR_MODE_BIT                7
#define MPU6050_TC_OFFSET_BIT                  6
#define MPU6050_TC_OFFSET_LENGTH               6
#define MPU6050_TC_OTP_BNK_VLD_BIT             0

#define MPU6050_VDDIO_LEVEL_VLOGIC             0
#define MPU6050_VDDIO_LEVEL_VDD                1

#define MPU6050_CFG_EXT_SYNC_SET_BIT           5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH        3
#define MPU6050_CFG_DLPF_CFG_BIT               2
#define MPU6050_CFG_DLPF_CFG_LENGTH            3

#define MPU6050_EXT_SYNC_DISABLED              0X0
#define MPU6050_EXT_SYNC_TEMP_OUT_L            0X1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L           0X2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L           0X3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L           0X4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L          0X5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L          0X6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L          0X7

#define MPU6050_DLPF_BW_256                    0X00
#define MPU6050_DLPF_BW_188                    0X01
#define MPU6050_DLPF_BW_98                     0X02
#define MPU6050_DLPF_BW_42                     0X03
#define MPU6050_DLPF_BW_20                     0X04
#define MPU6050_DLPF_BW_10                     0X05
#define MPU6050_DLPF_BW_5                      0X06

#define MPU6050_GCONFIG_FS_SEL_BIT             4
#define MPU6050_GCONFIG_FS_SEL_LENGTH          2

#define MPU6050_GYRO_FS_250                    0X00
#define MPU6050_GYRO_FS_500                    0X01
#define MPU6050_GYRO_FS_1000                   0X02
#define MPU6050_GYRO_FS_2000                   0X03

#define MPU6050_ACONFIG_XA_ST_BIT              7
#define MPU6050_ACONFIG_YA_ST_BIT              6
#define MPU6050_ACONFIG_ZA_ST_BIT              5
#define MPU6050_ACONFIG_AFS_SEL_BIT            4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH         2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT          2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH       3

#define MPU6050_ACCEL_FS_2                     0X00
#define MPU6050_ACCEL_FS_4                     0X01
#define MPU6050_ACCEL_FS_8                     0X02
#define MPU6050_ACCEL_FS_16                    0X03

#define MPU6050_DHPF_RESET                     0X00
#define MPU6050_DHPF_5                         0X01
#define MPU6050_DHPF_2P5                       0X02
#define MPU6050_DHPF_1P25                      0X03
#define MPU6050_DHPF_0P63                      0X04
#define MPU6050_DHPF_HOLD                      0X07

#define MPU6050_TEMP_FIFO_EN_BIT               7
#define MPU6050_XG_FIFO_EN_BIT                 6
#define MPU6050_YG_FIFO_EN_BIT                 5
#define MPU6050_ZG_FIFO_EN_BIT                 4
#define MPU6050_ACCEL_FIFO_EN_BIT              3
#define MPU6050_SLV2_FIFO_EN_BIT               2
#define MPU6050_SLV1_FIFO_EN_BIT               1
#define MPU6050_SLV0_FIFO_EN_BIT               0

#define MPU6050_MULT_MST_EN_BIT                7
#define MPU6050_WAIT_FOR_ES_BIT                6
#define MPU6050_SLV_3_FIFO_EN_BIT              5
#define MPU6050_I2C_MST_P_NSR_BIT              4
#define MPU6050_I2C_MST_CLK_BIT                3
#define MPU6050_I2C_MST_CLK_LENGTH             4

#define MPU6050_CLOCK_DIV_348                  0X0
#define MPU6050_CLOCK_DIV_333                  0X1
#define MPU6050_CLOCK_DIV_320                  0X2
#define MPU6050_CLOCK_DIV_308                  0X3
#define MPU6050_CLOCK_DIV_296                  0X4
#define MPU6050_CLOCK_DIV_286                  0X5
#define MPU6050_CLOCK_DIV_276                  0X6
#define MPU6050_CLOCK_DIV_267                  0X7
#define MPU6050_CLOCK_DIV_258                  0X8
#define MPU6050_CLOCK_DIV_500                  0X9
#define MPU6050_CLOCK_DIV_471                  0XA
#define MPU6050_CLOCK_DIV_444                  0XB
#define MPU6050_CLOCK_DIV_421                  0XC
#define MPU6050_CLOCK_DIV_400                  0XD
#define MPU6050_CLOCK_DIV_381                  0XE
#define MPU6050_CLOCK_DIV_364                  0XF

#define MPU6050_I2C_SLV_RW_BIT                 7
#define MPU6050_I2C_SLV_ADDR_BIT               6
#define MPU6050_I2C_SLV_ADDR_LENGTH            7
#define MPU6050_I2C_SLV_EN_BIT                 7
#define MPU6050_I2C_SLV_BYTE_SW_BIT            6
#define MPU6050_I2C_SLV_REG_DIS_BIT            5
#define MPU6050_I2C_SLV_GRP_BIT                4
#define MPU6050_I2C_SLV_LEN_BIT                3
#define MPU6050_I2C_SLV_LEN_LENGTH             4

#define MPU6050_I2C_SLV4_RW_BIT                7
#define MPU6050_I2C_SLV4_ADDR_BIT              6
#define MPU6050_I2C_SLV4_ADDR_LENGTH           7
#define MPU6050_I2C_SLV4_EN_BIT                7
#define MPU6050_I2C_SLV4_INT_EN_BIT            6
#define MPU6050_I2C_SLV4_REG_DIS_BIT           5
#define MPU6050_I2C_SLV4_MST_DLY_BIT           4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH        5

#define MPU6050_MST_PASS_THROUGH_BIT           7
#define MPU6050_MST_I2C_SLV4_DONE_BIT          6
#define MPU6050_MST_I2C_LOST_ARB_BIT           5
#define MPU6050_MST_I2C_SLV4_NACK_BIT          4
#define MPU6050_MST_I2C_SLV3_NACK_BIT          3
#define MPU6050_MST_I2C_SLV2_NACK_BIT          2
#define MPU6050_MST_I2C_SLV1_NACK_BIT          1
#define MPU6050_MST_I2C_SLV0_NACK_BIT          0

#define MPU6050_INTCFG_INT_LEVEL_BIT           7
#define MPU6050_INTCFG_INT_OPEN_BIT            6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT        5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT        4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT     3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT        2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT       1
#define MPU6050_INTCFG_CLKOUT_EN_BIT           0

#define MPU6050_INTMODE_ACTIVEHIGH             0X00
#define MPU6050_INTMODE_ACTIVELOW              0X01

#define MPU6050_INTDRV_PUSHPULL                0X00
#define MPU6050_INTDRV_OPENDRAIN               0X01

#define MPU6050_INTLATCH_50USPULSE             0X00
#define MPU6050_INTLATCH_WAITCLEAR             0X01

#define MPU6050_INTCLEAR_STATUSREAD            0X00
#define MPU6050_INTCLEAR_ANYREAD               0X01

#define MPU6050_INTERRUPT_FF_BIT               7
#define MPU6050_INTERRUPT_MOT_BIT              6
#define MPU6050_INTERRUPT_ZMOT_BIT             5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT       4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT      3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT      2
#define MPU6050_INTERRUPT_DMP_INT_BIT          1
#define MPU6050_INTERRUPT_DATA_RDY_BIT         0

#define MPU6050_DMPINT_5_BIT                   5
#define MPU6050_DMPINT_4_BIT                   4
#define MPU6050_DMPINT_3_BIT                   3
#define MPU6050_DMPINT_2_BIT                   2
#define MPU6050_DMPINT_1_BIT                   1
#define MPU6050_DMPINT_0_BIT                   0

#define MPU6050_MOTION_MOT_XNEG_BIT            7
#define MPU6050_MOTION_MOT_XPOS_BIT            6
#define MPU6050_MOTION_MOT_YNEG_BIT            5
#define MPU6050_MOTION_MOT_YPOS_BIT            4
#define MPU6050_MOTION_MOT_ZNEG_BIT            3
#define MPU6050_MOTION_MOT_ZPOS_BIT            2
#define MPU6050_MOTION_MOT_ZRMOT_BIT           0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT  7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT  4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT  3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT  2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT  1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT  0

#define MPU6050_PATHRESET_GYRO_RESET_BIT       2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT      1
#define MPU6050_PATHRESET_TEMP_RESET_BIT       0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT      5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH   2
#define MPU6050_DETECT_FF_COUNT_BIT            3
#define MPU6050_DETECT_FF_COUNT_LENGTH         2
#define MPU6050_DETECT_MOT_COUNT_BIT           1
#define MPU6050_DETECT_MOT_COUNT_LENGTH        2

#define MPU6050_DETECT_DECREMENT_RESET         0X0
#define MPU6050_DETECT_DECREMENT_1             0X1
#define MPU6050_DETECT_DECREMENT_2             0X2
#define MPU6050_DETECT_DECREMENT_4             0X3

#define MPU6050_USERCTRL_DMP_EN_BIT            7
#define MPU6050_USERCTRL_FIFO_EN_BIT           6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT        5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT        4
#define MPU6050_USERCTRL_DMP_RESET_BIT         3
#define MPU6050_USERCTRL_FIFO_RESET_BIT        2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT     1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT    0

#define MPU6050_PWR1_DEVICE_RESET_BIT          7
#define MPU6050_PWR1_SLEEP_BIT                 6
#define MPU6050_PWR1_CYCLE_BIT                 5
#define MPU6050_PWR1_TEMP_DIS_BIT              3
#define MPU6050_PWR1_CLKSEL_BIT                2
#define MPU6050_PWR1_CLKSEL_LENGTH             3

#define MPU6050_CLOCK_INTERNAL                 0X00
#define MPU6050_CLOCK_PLL_XGYRO                0X01
#define MPU6050_CLOCK_PLL_YGYRO                0X02
#define MPU6050_CLOCK_PLL_ZGYRO                0X03
#define MPU6050_CLOCK_PLL_EXT32K               0X04
#define MPU6050_CLOCK_PLL_EXT19M               0X05
#define MPU6050_CLOCK_KEEP_RESET               0X07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT          7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH       2
#define MPU6050_PWR2_STBY_XA_BIT               5
#define MPU6050_PWR2_STBY_YA_BIT               4
#define MPU6050_PWR2_STBY_ZA_BIT               3
#define MPU6050_PWR2_STBY_XG_BIT               2
#define MPU6050_PWR2_STBY_YG_BIT               1
#define MPU6050_PWR2_STBY_ZG_BIT               0

#define MPU6050_WAKE_FREQ_1P25                 0X0
#define MPU6050_WAKE_FREQ_2P5                  0X1
#define MPU6050_WAKE_FREQ_5                    0X2
#define MPU6050_WAKE_FREQ_10                   0X3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT          6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT      5
#define MPU6050_BANKSEL_MEM_SEL_BIT            4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH         5

#define MPU6050_WHO_AM_I_BIT                   6
#define MPU6050_WHO_AM_I_LENGTH                6

#define MPU6050_DMP_MEMORY_BANKS               8
#define MPU6050_DMP_MEMORY_BANK_SIZE           256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE          16

extern void MPU6050_GetFIFOBytes(u8 *data, u8 length);
extern void MPU6050_Init(void);
extern void MPU6050_ReadAcc(s16 *data);
extern void MPU6050_ReadGyr(s16 *data);
extern void MPU6050_ReadMemoryBlock(u8 *data, u16 data_size, u8 bank,
                                    u8 address);
extern void MPU6050_Reset(void);
extern void MPU6050_ResetDMP(void);
extern void MPU6050_ResetFIFO(void);
extern void MPU6050_ResetIICMaster(void);
extern void MPU6050_SetAccOffset(s16 offset[3]);
extern void MPU6050_SetClockSource(u8 source);
extern void MPU6050_SetDLPFMode(u8 mode);
extern void MPU6050_SetDMPConfig1(u8 config);
extern void MPU6050_SetDMPConfig2(u8 config);
extern void MPU6050_SetDMPEnabled(u8 enabled);
extern void MPU6050_SetExternalFrameSync(u8 sync);
extern void MPU6050_SetSleepEnabled(u8 enabled);
extern void MPU6050_SetMemoryBank(u8 bank, u8 prefetch_enable, u8 user_bank);
extern void MPU6050_SetMemoryStartAddress(u8 address);
extern void MPU6050_SetIntEnabled(u8 enabled);
extern void MPU6050_SetRate(u8 rate);
extern void MPU6050_SetFIFOEnabled(u8 enabled);
extern void MPU6050_SetFullScaleAccRange(u8 range);
extern void MPU6050_SetFullScaleGyrRange(u8 range);
extern void MPU6050_SetGyrOffset(s16 offset[3]);
extern void MPU6050_SetIICBypassEnabled(u8 enabled);
extern void MPU6050_SetIICMasterModeEnabled(u8 enabled);
extern void MPU6050_SetMotionDetectionDuration(u8 duration);
extern void MPU6050_SetMotionDetectionThreshold(u8 threshold);
extern void MPU6050_SetOTPBankValid(u8 enabled);
extern void MPU6050_SetSlaveAddress(u8 numbers, u8 address);
extern void MPU6050_SetXGyrOffset(s16 offset);
extern void MPU6050_SetXGyrOffsetTC(s8 offset);
extern void MPU6050_SetYGyrOffset(s16 offset);
extern void MPU6050_SetYGyrOffsetTC(s8 offset);
extern void MPU6050_SetZeroMotionDetectionDuration(u8 duration);
extern void MPU6050_SetZeroMotionDetectionThreshold(u8 threshold);
extern void MPU6050_SetZGyrOffset(s16 offset);
extern void MPU6050_SetZGyrOffsetTC(s8 offset);
extern u8   MPU6050_Check(void);
extern u8   MPU6050_GetDeviceID(void);
extern u8   MPU6050_GetIntStatus(void);
extern u8   MPU6050_GetOTPBankValid(void);
extern u8   MPU6050_IsDRY(void);
extern u8   MPU6050_ReadMemoryByte(void);
extern u8   MPU6050_TestConnection(void);
extern u8   MPU6050_WriteDMPConfigurationSet(uc8 *data, u16 data_size);
extern u8   MPU6050_WriteMemoryBlock(uc8 *data, u16 data_size, u8 bank,
                                     u8 address, u8 verify_flag);
extern u8   MPU6050_WriteProgMemoryBlock(uc8 *data, u16 data_size, u8 bank,
                                         u8 address, u8 verify_flag);
extern u8   MPU6050_WriteProgDMPConfigurationSet(uc8 *data, u16 data_size);
extern s8   MPU6050_GetXGyrOffsetTC(void);
extern s8   MPU6050_GetYGyrOffsetTC(void);
extern s8   MPU6050_GetZGyrOffsetTC(void);
extern u16  MPU6050_GetFIFOCount(void);
extern s16  MPU6050_GetXGyrOffset(void);
extern s16  MPU6050_GetYGyrOffset(void);
extern s16  MPU6050_GetZGyrOffset(void);

#endif
