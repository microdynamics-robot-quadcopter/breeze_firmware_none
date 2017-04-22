/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_nrf24l01.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: implement the nrf24l01 function
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

#include "stm32f10x_driver_spi.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_rpdata.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_it.h"
#include "stdio.h"

uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];  /* The received data of NRF24l01 */
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];  /* The data need to be transmit of NRF24l01 */

/* Modify this address of receiving and transmitting, can support many equipments using in same region */
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0xc3, 0x10, 0x10, 0x00};   /* Receiving address */

void NRF24L01_Init(void)
{
    SPI_INIT();

    /* Check if NRF24L01 is in the SPI bus */
    NRF24L01_Check();

    /* Set the NRF RX address, priority to the latest address in eeprom */
    NRF_Matching();
}

void SetRX_Mode(void)
{
    SPI_CE_L();
    NRF_Write_Reg(FLUSH_RX, 0xff);                                                   /* Clear TX FIFO register */
    NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*) RX_ADDRESS, RX_ADR_WIDTH);  /* Write RX node address */
    NRF_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);                                      /* Enable the automatic response of channel 0 */
    NRF_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);                                  /* Enable the receiving address of channel 0 */
    NRF_Write_Reg(NRF_WRITE_REG + RF_CH, 40);                                        /* Set the communication frequency of RF */
    NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                         /* Select the valid data width of channel 0 */
    NRF_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);                                   /* Set parameters of TX: 0db, 2Mbps, enable low noise gain */
    NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);                                     /* Set parameters: PWR_UP, EN_CRC, 16BIT_CRC, receive mode */
    SPI_CE_H();
    // printf("NRF24L01 Set to Receiving Mode,RX_ADDR 0x%x...\r\n",RX_ADDRESS[4]);
}

void NRF_TxPacket(uint8_t *tx_buf, uint8_t len)
{
    SPI_CE_L();                               /* StandBy I mode */
    NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len);  /* Load data */
    SPI_CE_H();                               /* Set high CE, enable data transmitting */
}

uint8_t NRF_Write_Reg(uint8_t reg, uint8_t val)
{
    uint8_t status;
    SPI_CSN_L();
    status = SPI_RW(reg);
    SPI_RW(val);            /* Write data */
    SPI_CSN_H();            /* Disable this component */
    return status;
}

uint8_t NRF_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    SPI_CSN_L();
    SPI_RW(reg);
    reg_val = SPI_RW(0);    /* Read data */
    SPI_CSN_H();
    return reg_val;
}

uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();              /* Enable this component */
    status = SPI_RW(reg);     /* Write register address */

    for (i = 0; i < len; i++)
    {
        pBuf[i] = SPI_RW(0);  /* Read data */
    }

    SPI_CSN_H();              /* Disable this component */
    return status;
}

uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();           /* Enable this component */
    status = SPI_RW(reg);  /* Write register address */

    for (i = 0; i < len; i++)
    {
        SPI_RW(pBuf[i]);   /* Write data */
    }

    SPI_CSN_H();           /* Disable this component */
    return status;
}

/* Query interrupt */
void NRF_Irq(void)
{
    uint8_t status = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
    if (status & (1 << RX_DR))                                       /* Receive the flag of polling */
    {
        NRF_Read_Buf(RD_RX_PLOAD, NRF24L01_RXDATA, RX_PLOAD_WIDTH);  /* Read receive payload from RX_FIFO buffer */
        ReceiveDataFromNRF();                                        /* Can be modified */
        NRF_Write_Reg(0x27, status);                                 /* Clear the interrupt flag of nrf */
        status = 0;
    }
}

/* Receive data */
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
    u8 status;

    //SPI2_SetSpeed(SPI_SPEED_4);                          /* The speed of spi is 9Mhz(maximum is 10Mhz) */
    status = NRF_Read_Reg(NRFRegSTATUS);                   /* Read the value of status register */
    NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, status);   /* Clear the interrupt flag of TX_DS or MAX_RT */

    if (status & RX_OK)                                    /* Have received data */
    {
        NRF_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);  /* Read data */
        NRF_Write_Reg(FLUSH_RX, 0xff);                     /* Clear RX FIFO register */
        return 0;
    }
    return 1;                                              /* Never receive any data */
}

/* Determine whether the SPI interface access to the NRF chip is available */
u8 NRF24L01_Check(void)
{
    u8 i;
    u8 buf1[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
    u8 buf2[5];

    NRF_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf1, 5);       /* Write 5 bytes address */
    NRF_Read_Buf(TX_ADDR, buf2, 5);                        /* Read address that have been written */

    /* Compare */
    for (i = 0; i < 5; i++)
    {
        if (buf2[i] != 0xC2)
        {
            break;
        }
    }

    if (i == 5)
    {
        // printf("NRF24L01 found...\r\n"); return 1;
    }         /* MCU connects to NRF successfully */
    else
    {
        // printf("NRF24L01 check failed...\r\n"); return 0;
    }
        /* MCU connects to NRF unsuccessfully */
}

static uint8_t sta;
extern void SaveParamsToEEPROM(void);
u8 NRFMatched = 0;

void NRF_Matching(void)
{
    static uint32_t nTs, nT;
    static uint32_t writeOvertime = 2 * 1000000;  /* Unit: us */

    LED_A_ON;
    LED_B_ON;
    LED_C_ON;
    LED_D_ON;
    nTs = Delay_GetRuntimeUs();

    do
    {
        NRFMatched = 0;
        nT = Delay_GetRuntimeUs() - nTs;

        if (nT >= writeOvertime)
        {
            RX_ADDRESS[4] = EEPROM_TableStructure.nrf_addr[4];
            break;    /* Exit when time out, and do not change original address */
        }

        SetRX_Mode(); /* Reset RX mode write RX panel address */
        Delay_TimeMs(4);  /* Delay is needed after reset NRF */
        sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);

        if ((sta & 0x0E) == 0x00)
        {
            NRFMatched = 1;
        }
        else
        {
            RX_ADDRESS[4]++;   /* Search the next RX_ADDRESS */
            if (RX_ADDRESS[4] == 0xFF)
            {
                RX_ADDRESS[4] = 0x00;
            }
        }
    }
    while ((sta & 0x0E) == 0x0E);

    SetRX_Mode();              /* Reset RX mode */

    if ((NRFMatched == 1) && (RX_ADDRESS[4] != EEPROM_TableStructure.nrf_addr[4]))
    {
        SaveParamsToEEPROM();  /* Write eeprom when current addr != original addr */
    }

    LED_A_OFF;
    LED_B_OFF;
    LED_C_OFF;                  /* Matching end */
    LED_D_OFF;
}
