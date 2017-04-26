/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_flash.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.20
Description: Implement the flash operation function
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

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_flash.h"

// The start address of small capacity flash's last page is 0X08007C00, end
// address is 0X08007FFF. Every page is 1k bytes.

// Lock flash.
void Flash_Lock(void)
{
    FLASH->CR |= 1 << 7;
}

// Read specified numbers of data in specified start address.
void Flash_Read(u32 read_addr, u16 *buffer, u16 half_word_nums)
{
    u16 i;

    for (i = 0; i < half_word_nums; i++)
    {
        // Read two bytes.
        buffer[i]  = Flash_ReadHalfWord(read_addr);
        // Offset two bytes.
        read_addr += 2;
    }
}

// Unlock flash.
void Flash_Unlock(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

#if FLASH_WREN

#if FLASH_SIZE < 256
#define FLASH_SECTOR_SIZE 1024
#else
#define FLASH_SECTOR_SIZE 2048
#endif

// 2K bytes at most.
u16 Flash_Buffer[FLASH_SECTOR_SIZE / 2];

// Write specified numbers of data in specified start address.
void Flash_Write(u32 write_addr, u16 *buffer, u16 half_word_nums)
{
    u16 i;
    // The offset address in sector(16 bits).
    u16 sector_offset;
    // The remain address in sector(16 bits).
    u16 sector_remain;
    // Sector address.
    u32 sector_addr;
    // The remain address after remove 0X08000000.
    u32 offset_addr;

    if (write_addr <   FLASH_BASE
    || (write_addr >= (FLASH_BASE + 1024 * FLASH_SIZE)))
    {
        // Illegal address.
        return ;
    }

    Flash_Unlock();

    // The actual offset address.
    offset_addr   =  write_addr - FLASH_BASE;
    sector_addr   =  offset_addr / FLASH_SECTOR_SIZE;
    // The offset address in sector(unit: 2 bytes).
    sector_offset = (offset_addr % FLASH_SECTOR_SIZE) / 2;
    // The remain range in sector.
    sector_remain = FLASH_SECTOR_SIZE / 2 - sector_offset;

    if (half_word_nums <= sector_remain)
    {
        sector_remain = half_word_nums;
    }

    while (1)
    {
        // Read data from whole sector.
        Flash_Read(sector_addr * FLASH_SECTOR_SIZE + FLASH_BASE, Flash_Buffer,
                   FLASH_SECTOR_SIZE / 2);
        // Check data.
        for (i = 0; i < sector_remain; i++)
        {
            // Need to erase.
            if (Flash_Buffer[sector_offset + i] != 0XFFFF)
            {
                break;
            }
        }
        // Need to erase.
        if (i < sector_remain)
        {
            // Erase the whole sector.
            Flash_ErasePage(sector_addr * FLASH_SECTOR_SIZE + FLASH_BASE);
            // Copy the whole sector.
            for (i = 0; i < sector_remain; i++)
            {
                Flash_Buffer[sector_offset + i] = buffer[i];
            }
            // Write data to whole sector.
            Flash_WriteNoCheck(sector_addr * FLASH_SECTOR_SIZE + FLASH_BASE,
                                Flash_Buffer, FLASH_SECTOR_SIZE / 2);
        }
        else
        {
            // Write data that be erased to remain sector.
            Flash_WriteNoCheck(write_addr, buffer, sector_remain);
        }
        // Writing is over.
        if (half_word_nums == sector_remain)
        {
            break;
        }
        else
        {
            sector_addr    += 1;
            sector_offset   = 0;
            buffer         += sector_remain;
            write_addr     += sector_remain;
            half_word_nums -= sector_remain;
            if (half_word_nums > (FLASH_SECTOR_SIZE / 2))
            {
                // The next sector cannot be writen fully.
                sector_remain = FLASH_SECTOR_SIZE / 2;
            }
            else
            {
                // The next sector can be writen fully.
                sector_remain = half_word_nums;
            }
        }
    }

    Flash_Lock();
}

void Flash_WriteNoCheck(u32 write_addr, u16 *buffer, u16 half_word_nums)
{
    u16 i;

    for (i = 0; i < half_word_nums; i++)
    {
        Flash_WriteHalfWord(write_addr, buffer[i]);
        write_addr += 2;
    }
}

#endif

u8 Flash_ErasePage(u32 page_addr)
{
    u8 status = 0;
    // Wait for the end of last operation(>20ms).
    status = Flash_WaitDone(0X5FFF);

    if (status == FLASH_STATE_OK)
    {
        // Erase page.
        FLASH->CR |= 1 << 1;
        // Set the page address.
        FLASH->AR  = page_addr;
        // Start to erase page.
        FLASH->CR |= 1 << 6;
        // Wait for the end of last operation(>20ms).
        status = Flash_WaitDone(0X5FFF);
        if (status != FLASH_STATE_BUSY)
        {
            // Clear the flag of erasing page.
            FLASH->CR &= ~(1 << 1);
        }
    }

    return status;
}

u8 Flash_GetStatus(void)
{
    u32 status = FLASH->SR;

    if (status & (1 << 0))
    {
        // Busy.
        return FLASH_STATE_BUSY;
    }
    else if (status & (1 << 2))
    {
        // Programming error.
        return FLASH_STATE_ERROR_PRO;
    }
    else if (status & (1 << 4))
    {
        // Write-protect error.
        return FLASH_STATE_ERROR_WP;
    }

    return 0;
}

u8 Flash_WaitDone(u16 time)
{
    u8 status;

    do
    {
        status = Flash_GetStatus();
        if (status != FLASH_STATE_BUSY)
        {
            break;
        }
        Delay_TimeUs(1);
        time--;
    }
    while (time);

    if (time == 0)
    {
        status = FLASH_STATE_TIMEOUT;
    }

    return status;
}

// Write half word in specified address of flash.
u8 Flash_WriteHalfWord(u32 flash_addr, u16 half_word)
{
    u8 status = Flash_WaitDone(0XFF);

    if (status == FLASH_STATE_OK)
    {
        // Enable programming.
        FLASH->CR |= 1 << 0;
        // Write half word.
        *(vu16 *)flash_addr = half_word;
        // Wait for the end of operation.
        status = Flash_WaitDone(0XFF);
        if (status != FLASH_STATE_BUSY)
        {
            // Clear PG bit.
            FLASH->CR &= ~(1 << 0);
        }
    }

    return status;
}

u16 Flash_ReadHalfWord(u32 flash_addr)
{
    return *(vu16 *)flash_addr;
}
