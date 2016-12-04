/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT 
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_driver_flash.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.8.20
Description: implement the flash operation function
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

#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_flash.h"

/* STM32F103T8U6--->64k bytes flash */
/* The start address of small capacity flash's last page is 0x08007c00, */
/* end address is 0x08007fff */
/* Every page is 1k bytes */
void STMFLASH_Unlock(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

void STMFLASH_Lock(void)
{
    FLASH->CR |= 1<<7;
}

u8 STMFLASH_GetStatus(void)
{
    u32 res;
    res = FLASH->SR;
    if (res & (1<<0))      /* Busy */
    {
        return 1;
    }
    else if (res & (1<<2)) /* Programming error */
    {
        return 2;
    }
    else if (res & (1<<4)) /* Write-protect error */
    {
        return 3;
    }
    return 0;
}

/* time: delay time */
/* Return: state */
u8 STMFLASH_WaitDone(u16 time)
{
    u8 res;
    do
    {
        res = STMFLASH_GetStatus();
        if (res != 1)
        {
            break;    /* Not busy */
        }
        delay_us(1);
        time--;
    }
    while (time);

    if (time == 0)
    {
        res = 0xFF;   /* Timeout */
    }
    return res;
}

/* paddr: page address */
/* Return: state */
u8 STMFLASH_ErasePage(u32 paddr)
{
    u8 res = 0;
    res = STMFLASH_WaitDone(0X5FFF);     /* Wait for the end of last operation(>20ms) */
    if (res == 0)
    {
        FLASH->CR |= 1<<1;               /* Erase page */
        FLASH->AR  = paddr;              /* Set the page address */
        FLASH->CR |= 1<<6;               /* Start erasing page */
        res = STMFLASH_WaitDone(0X5FFF); /* Wait for the end of operation(>20ms) */
        if (res != 1)                    /* Not Busy */
        {
            FLASH->CR &= ~(1<<1);        /* Clear the flag of erasing page */
        }
    }
    return res;
}

/* faddr: specified address(must be the multiple of 2) */
/* dat: data to be writen */
/* Return: state */
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat)
{
    u8 res;
    res = STMFLASH_WaitDone(0XFF);
    if (res == 0)                      /* OK */
    {
        FLASH->CR |= 1<<0;             /* Enable programming */
        *(vu16*)faddr = dat;           /* Write data */
        res = STMFLASH_WaitDone(0XFF); /* Wait for the end of operation */
        if (res != 1)                  /* Operation is finish */
        {
            FLASH->CR &= ~(1<<0);      /* Clear PG bit */
        }
    }
    return res;
} 

/* faddr: read address(must be the multiple of 2) */
/* Return: data to be read */
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16*)faddr;
}

#if STM32_FLASH_WREN  /* Enable the flag of writing */

/* WriteAddr: start address */
/* pBuffer: data pointer */
/* NumToWrite: halfword(16 bits) */
void STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u16 i;
    for (i = 0; i < NumToWrite; i++)
    {
        STMFLASH_WriteHalfWord(WriteAddr,pBuffer[i]);
        WriteAddr += 2;
    }
}

#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024  /* bytes */
#else
#define STM_SECTOR_SIZE 2048
#endif

u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];  /* 2k bytes(the most) */

/* WriteAddr: start address(must be the multiple of 2) */
/* pBuffer: data pointer */
/* NumToWrite: the number of data */
void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
    u32 secpos;    /* Sector address */
    u16 secoff;    /* The offset address in sector(16 bits) */
    u16 secremain; /* The remain address in sector(16 bits) */
    u16 i;
    u32 offaddr;   /* Remove the address that after 0X08000000 */
    if (WriteAddr < STM32_FLASH_BASE 
        || (WriteAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
    {
        return;    /* Illegal address */
    }

    STMFLASH_Unlock();
    offaddr = WriteAddr - STM32_FLASH_BASE;   /* The actual offset address */
    secpos = offaddr / STM_SECTOR_SIZE;
    secoff = (offaddr % STM_SECTOR_SIZE) / 2; /* The offset in sector(2 bytes) */
    secremain = STM_SECTOR_SIZE / 2 - secoff; /* The remain room in sector */

    if (NumToWrite <= secremain)
    {
        secremain = NumToWrite;
    }

    while (1)
    {
        STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE,
                      STMFLASH_BUF, STM_SECTOR_SIZE / 2);  /* Read the data in whole sector */
        for (i = 0; i < secremain; i++)                    /* Check data */
        {
            if (STMFLASH_BUF[secoff+i] != 0xFFFF)          /* Need to erase */
            {
                break;
            }
        }

        if (i < secremain)  /* Need to erase */
        {
            STMFLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);  /* Erase the whole sector */
            for (i = 0; i < secremain; i++)                                   /* Copy */
            {
                STMFLASH_BUF[secoff+i] = pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE,
                                   STMFLASH_BUF, STM_SECTOR_SIZE / 2); /* Write the data in whole sector */
        }
        else
        {
            STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);     /* Write the data that be erased */
        }

        if (NumToWrite == secremain) /* Writing is over */
        {
            break;
        }
        else
        {
            secpos++;
            secoff      = 0;
            pBuffer    += secremain;
            WriteAddr  += secremain;
            NumToWrite -= secremain;
            if (NumToWrite > (STM_SECTOR_SIZE / 2))
            {
                secremain = STM_SECTOR_SIZE / 2;     /* The next sector cannot be writen fully */
            }
            else
            {
                secremain = NumToWrite;              /* The next sector can be writen fully */
            }
        }
    }
    STMFLASH_Lock();
}
#endif

/* ReadAddr: start address */
/* pBuffer: data pointer */
/* NumtoWrite: halfword(16 bits) */
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead)
{
    u16 i;
    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);  /* Read 2 bytes */
        ReadAddr += 2;
    }
}
