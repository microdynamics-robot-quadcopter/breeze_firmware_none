#ifndef __STM32F10X_DRIVER_FLASH_H__
#define __STM32F10X_DRIVER_FLASH_H__

#include "stm32f10x.h"

/*用户根据自己的需求设置*/
#define STM32_FLASH_SIZE 64            /*所选STM32的FLASH容量(单位:k)*/
#define STM32_FLASH_WREN 1             /*使能FLASH写入(0:不使能; 1:使能)*/

#define STM32_FLASH_BASE   0x08000000  /*STM32 FlASH的起始地址*/
#define STM32_FLASH_OFFEST 0x0000fc00  /*人为定义Flash相对起始地址的偏移量*/

/*FLASH解锁键值*/
#define FLASH_KEY1         0X45670123
#define FLASH_KEY2         0XCDEF89AB

extern void STMFLASH_Unlock(void);                                           /*FLASH解锁*/
extern void STMFLASH_Lock(void);                                             /*FLASH上锁*/
extern u8 STMFLASH_GetStatus(void);                                          /*获得状态*/
extern u8 STMFLASH_WaitDone(u16 time);                                       /*等待操作结束*/
extern u8 STMFLASH_ErasePage(u32 paddr);                                     /*擦除页*/
extern u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);                        /*写入半字*/
extern u16 STMFLASH_ReadHalfWord(u32 faddr);                                 /*读出半字*/
extern void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite);     /*从指定地址开始写入指定长度的数据*/
extern void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead);        /*从指定地址开始读出指定长度的数据*/

#endif
