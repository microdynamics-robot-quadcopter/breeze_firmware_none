/*******************************************************************************
Copyright (C), 2016-2016, Team MicroDynamics. 

Filename:    stm32f10x_driver_iic.c
Author:      maksyuki
Version:     1.0
Date:        2016.8.3
Description: implement the IIC operation function
Others:      none
Function List:
             1. void IIC_Init(void); 
             2. void IIC_Start(void);
             3. void IIC_Stop(void);
			 4. void IIC_Send_Byte(u8 txd);
			 5. u8 IIC_Read_Byte(u8 ack);
			 6. u8 IIC_Wait_Ack(void); 
			 7. void IIC_Ack(void); 
			 8. void IIC_NAck(void);  
History:     none
*******************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_system_delay.h"
#include "stm32f10x_driver_iic.h"

/*******************************************************************************
Function:       void IIC_Init(void)
Description:    initializes the IIC
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          none
Output:         none
Return:         none
Others:         none
*******************************************************************************/
void IIC_Init(void)
{					     
	GPIO_SetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);
}

/*******************************************************************************
Function:       void IIC_Start(void)
Description:    send start signal
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          none
Output:         none
Return:         none
Others:         none
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     	 /*Output SDA*/
	IIC_SDA_H;
	IIC_SCL_H;
	delay_us(4);
 	IIC_SDA_L;
	IIC_SCL_L;	    /*Lock bus to prepare to send or read data*/
}

/*******************************************************************************
Function:       void IIC_Stop(void)
Description:    send end signal
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          none
Output:         none
Return:         none
Others:         none
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();		 /*Output SDA*/
	IIC_SDA_L;
 	delay_us(4);
	IIC_SCL_H;
	IIC_SDA_H;		 /*Send end signal*/
	delay_us(5);     /*Restarting needs 4.7us*/
}

/*******************************************************************************
Function:       void IIC_Send_Byte(u8 txd)
Description:    IIC sends one byte
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          
Output:         none
Return:         none
Others:         none
*******************************************************************************/
void IIC_Send_Byte(u8 txd)
{               
    u8 i;    
	SDA_OUT(); 	    
    IIC_SCL_L;		/*Prepare to send data*/   
    
    for (i = 0; i < 8; i++)
    {              
		if ((txd & 0x80) >> 7)
		{
			IIC_SDA_H;
		}
		else
		{
			IIC_SDA_L;
		}
				
		txd <<= 1; 	  
		IIC_SCL_H;
		IIC_SCL_L;
    }
	IIC_Wait_Ack();
}

/*******************************************************************************
Function:       u8 IIC_Read_Byte(u8 ack)
Description:    IIC reads one byte
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          
Output:         none
Return:         
Others:         none
*******************************************************************************/
u8 IIC_Read_Byte(u8 ack)
{
	u8 i, receive = 0;
	SDA_IN();	            /*Set the SDA*/
    
    for (i = 0; i < 8; i++)
	{
        IIC_SCL_L; 
		IIC_SCL_H;
		receive <<= 1;
		if (READ_SDA)		/*Receive 1*/
        {
            receive++;
        }
    }		
    
	if (!ack)
    {
        IIC_NAck();		    /*Send NACK signal*/
    }
	else
    {
        IIC_Ack(); 		    /*Send ACK signal*/
    }
	return receive;
}

/*******************************************************************************
Function:       u8 IIC_Wait_Ack(void)
Description:    IIC waits ACK signal
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          none
Output:         none
Return:         
Others:         none
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SDA_IN();      				/*Set the SDA*/
	IIC_SDA_H;	   
	IIC_SCL_H;
	 
	while (READ_SDA)			/*SDA is high level, wait to push down*/
	{
        ucErrTime++;
		if (ucErrTime > 250)	/*If after 40*250=1ms the IIC is no response, it will send end signal*/
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;
	return 0;
}

/*******************************************************************************
Function:       void IIC_Ack(void)
Description:    IIC sends ACK signal
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          none
Output:         none
Return:         none
Others:         none
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_L;
	IIC_SCL_H;
	IIC_SCL_L;
}

/*******************************************************************************
Function:       void IIC_NAck(void)
Description:    IIC doesn't send ACK signal
Calls:          
Called By:      
Table Accessed: none
Table Updated:  none
Input:          none
Output:         none
Return:         none
Others:         none
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_H;
	IIC_SCL_H;
	IIC_SCL_L;
}
