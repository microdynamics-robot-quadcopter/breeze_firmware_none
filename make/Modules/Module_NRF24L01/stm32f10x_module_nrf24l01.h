/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_nrf24l01.h
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: declare the nrf24l01 function
Others:      none
Function List:
             1. void NRF24L01_Init(void);
             2. void SetRX_Mode(void);
             3. void SetTX_Mode(void);
             4. void NRF_TxPacket(uint8_t *tx_buf, uint8_t len);
             5. uint8_t NRF_RxPacket(uint8_t *rx_buf, uint8_t len);
             6. u8 NRF24L01_RxPacket(u8 *rxbuf);
             7. uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value);
             8. uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
             9. uint8_t NRF_Read_Reg(uint8_t reg);
            10. uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
            11. void NRF_Irq(void);
            12. u8 NRF24L01_Check(void);
            13. void NRF_Matching(void);
History:
1. <author>    <date>         <desc>
   maksyuki  2017.01.10  modify the module
*******************************************************************************/

#ifndef __STM32F10X_MODULE_NRF24L01_H__
#define __STM32F10X_MODULE_NRF24L01_H__

#include "stm32f10x.h"

//***************************************NRF24L01*******************************
#define TX_ADR_WIDTH    5       /* 5 uints TX address width */
#define RX_ADR_WIDTH    5       /* 5 uints RX address width */

#define RX_PLOAD_WIDTH  32      /* 32 uints TX payload */
#define TX_PLOAD_WIDTH  32      /* 32 uints TX payload */
//***************************************NRF24L01 register instructions*********
#define NRF_READ_REG    0x00    /* Read register */
#define NRF_WRITE_REG   0x20    /* Write register */
#define RD_RX_PLOAD     0x61    /* Read receiving data */
#define WR_TX_PLOAD     0xA0    /* Write transmitting data */
#define FLUSH_TX        0xE1    /* Flash transmitting FIFO */
#define FLUSH_RX        0xE2    /* Flash receiving FIFO */
#define REUSE_TX_PL     0xE3    /* Reload data */
#define NOP             0xFF    /* Reserve */
//*************************************SPI(nRF24L01) registers address**********
#define CONFIG          0x00    /* Configure */
#define EN_AA           0x01    /* Set the automatic response */
#define EN_RXADDR       0x02    /* Set the available channel */
#define SETUP_AW        0x03    /* Set the width of receiving and transmitting's address */
#define SETUP_RETR      0x04    /* Set the automatic resend */
#define RF_CH           0x05    /* Set the working frequency */
#define RF_SETUP        0x06    /* Set the transmitting rate, power */
#define NRFRegSTATUS    0x07    /* Status register */
#define OBSERVE_TX      0x08    /* Transmission monitoring */
#define CD              0x09    /* Address detection */
#define RX_ADDR_P0      0x0A    /* The receiving address of channel 0 */
#define RX_ADDR_P1      0x0B    /* The receiving address of channel 1 */
#define RX_ADDR_P2      0x0C    /* The receiving address of channel 2 */
#define RX_ADDR_P3      0x0D    /* The receiving address of channel 3 */
#define RX_ADDR_P4      0x0E    /* The receiving address of channel 4 */
#define RX_ADDR_P5      0x0F    /* The receiving address of channel 5 */
#define TX_ADDR         0x10    /* Transmitting address register */
#define RX_PW_P0        0x11    /* The receiving data's length of channel 0 */
#define RX_PW_P1        0x12    /* The receiving data's length of channel 1 */
#define RX_PW_P2        0x13    /* The receiving data's length of channel 2 */
#define RX_PW_P3        0x14    /* The receiving data's length of channel 3 */
#define RX_PW_P4        0x15    /* The receiving data's length of channel 4 */
#define RX_PW_P5        0x16    /* The receiving data's length of channel 5 */
#define FIFO_STATUS     0x17    /* Set the status register of FIFO */
//*****************************************************************************
#define RX_DR           6       /* The flag of interrupt */
#define TX_DS           5
#define MAX_RT          4

#define MAX_TX          0x10   /* Maximum number of transmissions interrupt */
#define TX_OK           0x20   /* Transmitting data interrupt */
#define RX_OK           0x40   /* Receiving data interrupt */

extern void NRF24L01_Init(void);
extern void SetRX_Mode(void);
extern void SetTX_Mode(void);
extern void NRF_TxPacket(uint8_t *tx_buf, uint8_t len);
extern uint8_t NRF_RxPacket(uint8_t *rx_buf, uint8_t len);
extern u8 NRF24L01_RxPacket(u8 *rxbuf);
extern uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value);
extern uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
extern uint8_t NRF_Read_Reg(uint8_t reg);
extern uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);

extern void NRF_Irq(void);
extern u8 NRF24L01_Check(void);

extern u8 NRFMatched;
extern uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];
extern uint8_t NRF24L01_TXDATA[TX_PLOAD_WIDTH];

extern void NRF_Matching(void);

#endif
