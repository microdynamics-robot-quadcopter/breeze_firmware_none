/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_flash.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.20
Description: declare the flash operation function
Others:      none
Function List:
             1. void STMFLASH_Lock(void);
             2. void STMFLASH_Unlock(void);
             3. u8 STMFLASH_GetStatus(void);
             4. u8 STMFLASH_WaitDone(u16 time);
             5. u8 STMFLASH_ErasePage(u32 paddr);
             6. u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);
             7. u16 STMFLASH_ReadHalfWord(u32 faddr);
             8. void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);
             9. void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);
History:
1. <author>    <date>         <desc>
   maksyuki  2016.12.04  modify the module
*******************************************************************************/

#ifndef __STM32F10X_DRIVER_FLASH_H__
#define __STM32F10X_DRIVER_FLASH_H__

#include "stm32f10x.h"

/* Custom the settings */
#define STM32_FLASH_SIZE 64            /* The capacity of flash(unit:k) */
#define STM32_FLASH_WREN 1             /* Enable flash writing(0:disable) */

#define STM32_FLASH_BASE   0x08000000  /* The start address */
#define STM32_FLASH_OFFEST 0x0000fc00  /* The relative offset to start addfress */

/* The unlock key value */
#define FLASH_KEY1         0X45670123
#define FLASH_KEY2         0XCDEF89AB

extern void STMFLASH_Lock(void);
extern void STMFLASH_Unlock(void);
extern u8 STMFLASH_GetStatus(void);
extern u8 STMFLASH_WaitDone(u16 time);
extern u8 STMFLASH_ErasePage(u32 paddr);
extern u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);
extern u16 STMFLASH_ReadHalfWord(u32 faddr);

/* Read the specified length data in specified start address */
extern void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);

/* Write the specified length data in specified start address */
extern void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);

#endif
