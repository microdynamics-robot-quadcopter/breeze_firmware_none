/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_flash.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.20
Description: Declare the flash operation function
Others:      none
Function List:
             1.  void Flash_Lock(void);
             2.  void Flash_Read(u32 read_addr, u16 *buffer,
                                 u16 half_word_nums);
             3.  void Flash_Unlock(void);
             4.  void Flash_Write(u32 write_addr, u16 *buffer,
                                  u16 half_word_nums);
             5.  void Flash_WriteNoCheck(u32 write_addr, u16 *buffer,
                                         u16 half_word_nums);
             6.  u8   Flash_ErasePage(u32 page_addr);
             7.  u8   Flash_GetStatus(void);
             8.  u8   Flash_WaitDone(u16 time);
             9.  u8   Flash_WriteHalfWord(u32 flash_addr, u16 half_word);
             10. u16  Flash_ReadHalfWord(u32 flash_addr);
History:
<author>    <date>        <desc>
maksyuki    2016.12.04    Modify the module
myyerrol    2017.04.23    Format the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_FLASH_H__
#define __STM32F10X_DRIVER_FLASH_H__

#include "stm32f10x.h"

// Custom the settings.
// The capacity of flash(unit:k).
#define FLASH_SIZE            64
// Enable flash writing(0:disable).
#define FLASH_WREN            1
// The relative offset to start addfress.
#define FLASH_OFFEST          0X0000FC00
// The unlock key value.
#define FLASH_KEY1            0X45670123
#define FLASH_KEY2            0XCDEF89AB

#define FLASH_STATE_OK        0
#define FLASH_STATE_BUSY      1
// Programming error.
#define FLASH_STATE_ERROR_PRO 2
// Write-protect error.
#define FLASH_STATE_ERROR_WP  3
#define FLASH_STATE_TIMEOUT   0XFF

extern void Flash_Lock(void);
extern void Flash_Read(u32 read_addr, u16 *buffer, u16 half_word_nums);
extern void Flash_Unlock(void);
extern void Flash_Write(u32 write_addr, u16 *buffer, u16 half_word_nums);
extern void Flash_WriteNoCheck(u32 write_addr, u16 *buffer, u16 half_word_nums);
extern u8   Flash_ErasePage(u32 page_addr);
extern u8   Flash_GetStatus(void);
extern u8   Flash_WaitDone(u16 time);
extern u8   Flash_WriteHalfWord(u32 flash_addr, u16 half_word);
extern u16  Flash_ReadHalfWord(u32 flash_addr);

#endif
