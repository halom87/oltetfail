/*
 * I2C.c
 *
 *  Created on: 2012.12.09.
 *      Author: Adam
 */
#include "I2C.h"
#include "DMA.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"


static xSemaphoreHandle I2CBusy;
xSemaphoreHandle I2C_TransferComplete;
xQueueHandle I2C_SendQueue;
xQueueHandle I2C_ReceiveQueue;

uint8_t I2C_TransmitNReceive; //1 transmit 0 receive
//
void I2C_Config(void)
{
	I2C_InitTypeDef I2C_InitStructure;

	I2CBusy = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(I2C_TransferComplete);
	xSemaphoreTake(I2C_TransferComplete,0);
	I2C_SendQueue=xQueueCreate(3,sizeof (uint8_t)); //Device address, Register address, data
	I2C_ReceiveQueue=xQueueCreate(6,sizeof (uint8_t)); //3x2 byte


	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	I2C_InitStructure.I2C_ClockSpeed=100000;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;

	I2C_Init(I2C2,&I2C_InitStructure);

	I2C_Cmd(I2C2,ENABLE);



}


uint8_t I2C_BufferRead(uint8_t* Data, uint8_t deviceAddress, uint8_t ReadAddr, uint8_t len)
{
	int i=0;
	if (xSemaphoreTake(I2CBusy,portMAX_DELAY))
	{
	/*	for (i=0; i<len; i++)
		{*/
			I2C_TransmitNReceive=0;
			while (I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));
			xQueueSend(I2C_SendQueue,&deviceAddress,0);
			if (len>1)ReadAddr|=0x80;
			xQueueSend(I2C_SendQueue,&ReadAddr,0);

			I2C_ClearFlag(I2C2,I2C_FLAG_AF);
			I2C_AcknowledgeConfig(I2C2, ENABLE);

			I2C_ITConfig(I2C2,I2C_IT_EVT,ENABLE);
			I2C_DMAReceive(Data,len);
			I2C_GenerateSTART(I2C2, ENABLE);
			xSemaphoreTake(I2C_TransferComplete,portMAX_DELAY);
			ReadAddr++;
			//xQueueReceive(I2C_ReceiveQueue,(Data + i),portMAX_DELAY);
	        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
	        while ((I2C2->CR1&0x200) == 0x200);

		//}

	 	xSemaphoreGive(I2CBusy);
	    return 1;
	}
	return 0;
}
uint8_t I2C_ByteWrite(uint8_t Data, uint8_t deviceAddress, uint8_t WriteAddr)
{
	if (xSemaphoreTake(I2CBusy,portMAX_DELAY))
	{
		I2C_TransmitNReceive=1;
		if (I2C_SendQueue==0)
		{
			return 0;
		}
		xQueueSend(I2C_SendQueue,&deviceAddress,0);
		xQueueSend(I2C_SendQueue,&WriteAddr,0);
		xQueueSend(I2C_SendQueue,&Data,0);


	    while (I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));
	    I2C_AcknowledgeConfig(I2C2, ENABLE);

	    I2C_ITConfig(I2C2,I2C_IT_EVT,ENABLE);

		I2C_GenerateSTART(I2C2, ENABLE);

		xSemaphoreTake(I2C_TransferComplete,portMAX_DELAY);
        /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
        while ((I2C2->CR1&0x200) == 0x200);


		xSemaphoreGive(I2CBusy);
	    return 1;
	}
	return 0;
}
