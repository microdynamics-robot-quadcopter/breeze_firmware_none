/*******************************************************************************
THIS PROGRAM IS FREE SOFTWARE. YOU CAN REDISTRIBUTE IT AND/OR MODIFY IT
UNDER THE TERMS OF THE GNU GPLV3 AS PUBLISHED BY THE FREE SOFTWARE FOUNDATION.

Copyright (C), 2016-2016, Team MicroDynamics <microdynamics@126.com>

Filename:    stm32f10x_module_nrf24l01.c
Author:      maksyuki
Version:     0.1.0.20161231_release
Create date: 2016.08.14
Description: Implement the NRF24L01 function
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

#include <stdio.h>
#include "stm32f10x_it.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_eeprom.h"
#include "stm32f10x_driver_spi.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_nrf24l01.h"

u8 nrf24l01_rx_data[NRF24L01_WIDTH_PAYLOAD_RX];
u8 nrf24l01_tx_data[NRF24L01_WIDTH_PAYLOAD_TX];

// NRF24L01 can support many quadcopters using in same region by modifying the
// address.
u8 nrf24l01_rx_address[NRF24L01_WIDTH_ADDR_RX] =
{
    0X34,
    0Xc3,
    0X10,
    0X10,
    0X00
};

bool nrf24l01_matched_flag = false;

void NRF24L01_Init(void)
{
    SPI_InitSPI();

    // Check if NRF24L01 is in the SPI bus.
    NRF24L01_CheckConnection();

    // Set the NRF24L01 RX address, priority to the latest address in eeprom.
    NRF24L01_MatchDevice();
}

void NRF24L01_IRQHandler(void)
{
    u8 status = NRF24L01_ReadRegister(NRF24L01_REG_RD +
                                      NRF24L01_ADDR_REG_STATUS);

    // Receive the flag of polling.
    if (status & (1 << NRF24L01_IT_RX_DR))
    {
        // Read receive payload from buffer.
        NRF24L01_ReadBuffer(NRF24L01_PAYLOAD_RD, nrf24l01_rx_data,
                            NRF24L01_WIDTH_PAYLOAD_RX);
        ReceiveDataFromNRF();
        // Clear the flag of interrupt.
        NRF24L01_WriteRegister(0X27, status);
        status = 0;
    }
}

void NRF24L01_MatchDevice(void)
{
    u8  status;
    u32 start_timestamp;
    u32 period;
    // Unit: us.
    u32 overtime = 2 * 1000000;

    LED_A_ON;
    LED_B_ON;
    LED_C_ON;
    LED_D_ON;

    start_timestamp = Delay_GetRuntimeUs();

    do
    {
        nrf24l01_matched_flag = false;
        period = Delay_GetRuntimeUs() - start_timestamp;
        if (period >= overtime)
        {
            // Exit when time out, and do not change original address.
            nrf24l01_rx_address[4] = EEPROM_TableStructure.nrf_addr[4];
            break;
        }
        // Reset RX mode.
        NRF24L01_SetRxMode();
        // Delay is needed after reset NRF24L01.
        Delay_TimeMs(4);
        status = NRF24L01_ReadRegister(NRF24L01_REG_RD +
                                       NRF24L01_ADDR_REG_STATUS);
        if ((status & 0X0E) == 0X00)
        {
            nrf24l01_matched_flag = true;
        }
        else
        {
            // Search the next address.
            nrf24l01_rx_address[4]++;
            if (nrf24l01_rx_address[4] == 0XFF)
            {
                nrf24l01_rx_address[4] = 0X00;
            }
        }
    }
    while ((status & 0X0E) == 0X0E);

    // Reset RX mode.
    NRF24L01_SetRxMode();

    if ((nrf24l01_matched_flag == true) &&
        (nrf24l01_rx_address[4] != EEPROM_TableStructure.nrf_addr[4]))
    {
        EEPROM_SaveParamsToEEPROM();
    }

    LED_A_OFF;
    LED_B_OFF;
    LED_C_OFF;
    LED_D_OFF;
}

void NRF24L01_SetRxMode(void)
{
    SPI_CE_L();
    // Clear RX FIFO register.
    NRF24L01_WriteRegister(NRF24L01_FLUSH_RX, 0XFF);
    // Write RX node address.
    NRF24L01_WriteBuffer(NRF24L01_REG_WR + NRF24L01_ADDR_RX_P0,
                        (u8 *)nrf24l01_rx_address, NRF24L01_WIDTH_ADDR_RX);
    // Enable the automatic ack of channel 0.
    NRF24L01_WriteRegister(NRF24L01_REG_WR + NRF24L01_ADDR_EN_AA, 0X01);
    // Enable the receiving address of channel 0.
    NRF24L01_WriteRegister(NRF24L01_REG_WR + NRF24L01_ADDR_EN_CH, 0X01);
    // Set the communication frequency of RF.
    NRF24L01_WriteRegister(NRF24L01_REG_WR + NRF24L01_ADDR_RF_CH, 40);
    // Select the valid data width of channel 0.
    NRF24L01_WriteRegister(NRF24L01_REG_WR + NRF24L01_ADDR_RX_PW_P0,
                           NRF24L01_WIDTH_PAYLOAD_RX);
    // Set parameters of TX: 0db, 2Mbps, enable low noise gain.
    NRF24L01_WriteRegister(NRF24L01_REG_WR + NRF24L01_ADDR_RF_SETUP, 0X0F);
    // Set parameters: PWR_UP, EN_CRC, 16BIT_CRC, receive mode.
    NRF24L01_WriteRegister(NRF24L01_REG_WR + NRF24L01_ADDR_CONFIG, 0X0F);
    SPI_CE_H();
}

void NRF24L01_SetTxMode(void)
{
}

void NRF24L01_WritePacket(u8 *buffer, u8 length)
{
    // Set low CE, enter StandBy I mode.
    SPI_CE_L();
    NRF24L01_WriteBuffer(NRF24L01_PAYLOAD_WR, buffer, length);
    // Set high CE, enable data transmitting.
    SPI_CE_H();
}

// Judge whether the SPI interface access to the NRF24L01 chip is available.
u8 NRF24L01_CheckConnection(void)
{
    u8 i;
    u8 buffer1[5] = {0XC2, 0XC2, 0XC2, 0XC2, 0XC2};
    u8 buffer2[5];

    NRF24L01_WriteBuffer(NRF24L01_REG_WR + NRF24L01_ADDR_TX, buffer1, 5);
    NRF24L01_ReadBuffer(NRF24L01_ADDR_TX, buffer2, 5);

    for (i = 0; i < 5; i++)
    {
        if (buffer2[i] != 0XC2)
        {
            return 0;
        }
    }

    return 1;
}

u8 NRF24L01_ReadBuffer(u8 reg, u8 *buffer, u8 length)
{
    u8 i;
    u8 status;

    // Enable the component.
    SPI_CSN_L();
    // Write register address.
    status = SPI_ReadAndWrite(reg);

    for (i = 0; i < length; i++)
    {
         // Read data.
        buffer[i] = SPI_ReadAndWrite(0);
    }

    // Disable the component.
    SPI_CSN_H();

    return status;
}

u8 NRF24L01_ReadRegister(u8 reg)
{
    u8 byte;

    SPI_CSN_L();
    SPI_ReadAndWrite(reg);
    // Read data.
    byte = SPI_ReadAndWrite(0);
    SPI_CSN_H();

    return byte;
}

u8 NRF24L01_ReadPacket(u8 *buffer)
{
    u8 status;

    // Read the value of status register.
    status = NRF24L01_ReadRegister(NRF24L01_ADDR_REG_STATUS);
    // Clear the flag of interrupt.
    NRF24L01_WriteRegister(NRF24L01_REG_WR + NRF24L01_ADDR_REG_STATUS, status);

    if (status & NRF24L01_IT_OK_RX)
    {
        // Read data.
        NRF24L01_ReadBuffer(NRF24L01_PAYLOAD_RD, buffer,
                            NRF24L01_WIDTH_PAYLOAD_RX);
        //  Clear RX FIFO register.
        NRF24L01_WriteRegister(NRF24L01_FLUSH_RX, 0XFF);
        return 1;
    }

    return 0;
}

u8 NRF24L01_WriteBuffer(u8 reg, u8 *buffer, u8 length)
{
    u8 i;
    u8 status;

    // Enable the component.
    SPI_CSN_L();
    // Write register address.
    status = SPI_ReadAndWrite(reg);

    for (i = 0; i < length; i++)
    {
        // Write data.
        SPI_ReadAndWrite(buffer[i]);
    }

    // Disable the component.
    SPI_CSN_H();

    return status;
}

u8 NRF24L01_WriteRegister(u8 reg, u8 byte)
{
    u8 status;

    SPI_CSN_L();
    status = SPI_ReadAndWrite(reg);
    // Write data.
    SPI_ReadAndWrite(byte);
    SPI_CSN_H();

    return status;
}
