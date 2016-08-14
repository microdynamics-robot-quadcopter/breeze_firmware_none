#include "stm32f10x_driver_spi.h"

void SPI_INIT(void)
{
    SPI_InitTypeDef SPI_InitStructure; 
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
     
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /*NRF_CE--PA12*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12; 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /*NRF_CSN--PA4*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4; 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);	
        
    SPI_CSN_H();
    
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex; 
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master; 
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b; 
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low; 
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge; 
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft; 
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB; 
    SPI_InitStructure.SPI_CRCPolynomial     = 7; 
    SPI_Init(SPI1, &SPI_InitStructure); 
    /* Enable SPI1 */ 
    SPI_Cmd(SPI1, ENABLE);
}

u8 SPI_RW(u8 data) 
{ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
    SPI_I2S_SendData(SPI1, data); 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);  
    return SPI_I2S_ReceiveData(SPI1); 
}
