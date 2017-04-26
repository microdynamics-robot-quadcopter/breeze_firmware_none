/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_nrf24l01.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Declare the NRF24L01 function
Others:      none
Function List:
             1.  void NRF24L01_Init(void);
             2.  void NRF24L01_IRQHandler(void);
             3.  void NRF24L01_MatchDevice(void);
             4.  void NRF24L01_SetRxMode(void);
             5.  void NRF24L01_SetTxMode(void);
             6.  void NRF24L01_WritePacket(u8 *buffer, u8 length);
             7.  u8   NRF24L01_CheckConnection(void);
             8.  u8   NRF24L01_ReadBuffer(u8 reg, u8 *buffer, u8 length);
             9.  u8   NRF24L01_ReadRegister(u8 reg);
             10. u8   NRF24L01_ReadPacket(u8 *buffer);
             11. u8   NRF24L01_WriteBuffer(u8 reg, u8 *buffer, u8 length);
             12. u8   NRF24L01_WriteRegister(u8 reg, u8 byte);
History:
<author>    <date>        <desc>
maksyuki    2017.01.10    Modify the module
myyerrol    2017.04.25    Format the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_NRF24L01_H__
#define __STM32F10X_MODULE_NRF24L01_H__

#include <stdbool.h>
#include "stm32f10x.h"

// NRF24L01 bit width.
// 5 uints width of address.
#define NRF24L01_WIDTH_ADDR_RX    5
#define NRF24L01_WIDTH_ADDR_TX    5

// 32 uints width of payload.
#define NRF24L01_WIDTH_PAYLOAD_RX 32
#define NRF24L01_WIDTH_PAYLOAD_TX 32

// NRF24L01 register instructions.
// Read register.
#define NRF24L01_REG_RD           0X00
// Write register.
#define NRF24L01_REG_WR           0X20
// Read data.
#define NRF24L01_PAYLOAD_RD       0X61
// Write data.
#define NRF24L01_PAYLOAD_WR       0XA0
// Flush transmitting.
#define NRF24L01_FLUSH_TX         0XE1
// Flush receiving.
#define NRF24L01_FLUSH_RX         0XE2
// Reload data.
#define NRF24L01_RELOAD_DATA      0XE3
// Reserve.
#define NRF24L01_NOP              0XFF

// NRF24L01 register addresses.
// Configure.
#define NRF24L01_ADDR_CONFIG      0X00
// Set the automatic ack.
#define NRF24L01_ADDR_EN_AA       0X01
// Set the available channel.
#define NRF24L01_ADDR_EN_CH       0X02
// Set the width of receiving and transmitting's address.
#define NRF24L01_ADDR_SETUP_WIDTH 0X03
// Set the automatic retransmission.
#define NRF24L01_ADDR_SETUP_RETX  0X04
// Set the working frequency.
#define NRF24L01_ADDR_RF_CH       0X05
// Set the transmitting rate, power.
#define NRF24L01_ADDR_RF_SETUP    0X06
// Status register.
#define NRF24L01_ADDR_REG_STATUS  0X07
// Transmission monitoring.
#define NRF24L01_ADDR_TX_MONITOR  0X08
// Address detection.
#define NRF24L01_ADDR_DETECTION   0X09
// The receiving address of channel 0.
#define NRF24L01_ADDR_RX_P0       0X0A
// The receiving address of channel 1.
#define NRF24L01_ADDR_RX_P1       0X0B
// The receiving address of channel 2.
#define NRF24L01_ADDR_RX_P2       0X0C
// The receiving address of channel 3.
#define NRF24L01_ADDR_RX_P3       0X0D
// The receiving address of channel 4.
#define NRF24L01_ADDR_RX_P4       0X0E
// The receiving address of channel 5.
#define NRF24L01_ADDR_RX_P5       0X0F
// Transmitting address register.
#define NRF24L01_ADDR_TX          0X10
// The receiving data's length of channel 0.
#define NRF24L01_ADDR_RX_PW_P0    0X11
// The receiving data's length of channel 1.
#define NRF24L01_ADDR_RX_PW_P1    0X12
// The receiving data's length of channel 2.
#define NRF24L01_ADDR_RX_PW_P2    0X13
// The receiving data's length of channel 3.
#define NRF24L01_ADDR_RX_PW_P3    0X14
// The receiving data's length of channel 4.
#define NRF24L01_ADDR_RX_PW_P4    0X15
// The receiving data's length of channel 5.
#define NRF24L01_ADDR_RX_PW_P5    0X16
// Set the status register of FIFO.
#define NRF24L01_ADDR_FIFO_STATUS 0X17

// NRF24L01 interrupts.
#define NRF24L01_IT_RX_DR         6
#define NRF24L01_IT_TX_DS         5
#define NRF24L01_IT_MAX_RT        4
#define NRF24L01_IT_MAX_TX        0X10
#define NRF24L01_IT_OK_TX         0X20
#define NRF24L01_IT_OK_RX         0X40

extern u8   nrf24l01_rx_address[NRF24L01_WIDTH_ADDR_RX];
extern u8   nrf24l01_rx_data[NRF24L01_WIDTH_PAYLOAD_RX];
extern u8   nrf24l01_tx_data[NRF24L01_WIDTH_PAYLOAD_TX];
extern bool nrf24l01_matched_flag;

extern void NRF24L01_Init(void);
extern void NRF24L01_IRQHandler(void);
extern void NRF24L01_MatchDevice(void);
extern void NRF24L01_SetRxMode(void);
extern void NRF24L01_SetTxMode(void);
extern void NRF24L01_WritePacket(u8 *buffer, u8 length);
extern u8   NRF24L01_CheckConnection(void);
extern u8   NRF24L01_ReadBuffer(u8 reg, u8 *buffer, u8 length);
extern u8   NRF24L01_ReadRegister(u8 reg);
extern u8   NRF24L01_ReadPacket(u8 *buffer);
extern u8   NRF24L01_WriteBuffer(u8 reg, u8 *buffer, u8 length);
extern u8   NRF24L01_WriteRegister(u8 reg, u8 byte);

#endif
