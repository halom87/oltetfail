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

//
void I2C_Config(void)
{
	I2C_InitTypeDef I2C_InitStructure;

	I2CBusy = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(I2C_TransferComplete);
	xSemaphoreTake(I2C_TransferComplete,0);


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
		for (i=0; i<len; i++)
		{
			while (I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));

			I2C_ClearFlag(I2C2,I2C_FLAG_AF);
			I2C_AcknowledgeConfig(I2C2, ENABLE);

			I2C_GenerateSTART(I2C2, ENABLE);
			while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

			I2C_Send7bitAddress(I2C2, deviceAddress, I2C_Direction_Transmitter);
			while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

			//if (len>1) ReadAddr|=0x80;
			I2C_SendData(I2C2, ReadAddr + i);
			while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

			I2C_GenerateSTART(I2C2,ENABLE);
			while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

			I2C_Send7bitAddress(I2C2, deviceAddress, I2C_Direction_Receiver);
			while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

			I2C_AcknowledgeConfig(I2C2,DISABLE);
			while (!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED));

			*(Data + i) =I2C_ReceiveData(I2C2);

			I2C_GenerateSTOP(I2C2, ENABLE);
			while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF)); // stop bit flag
		}

	 	xSemaphoreGive(I2CBusy);
	    return 1;
	}
	return 0;

	/*
	if (xSemaphoreTake(I2CBusy,portMAX_DELAY))
	{
	    while (I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));

	    I2C_ClearFlag(I2C2,I2C_FLAG_AF);
	    I2C_AcknowledgeConfig(I2C2, ENABLE);

		I2C_GenerateSTART(I2C2, ENABLE);
		while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

		I2C_Send7bitAddress(I2C2, deviceAddress, I2C_Direction_Transmitter);
	    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	    if (len>1) ReadAddr|=0x80;
	    I2C_SendData(I2C2, ReadAddr);
	    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	    I2C_GenerateSTART(I2C2,ENABLE);
		while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

	    I2C_Send7bitAddress(I2C2, deviceAddress, I2C_Direction_Receiver);
	    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

		I2C_AcknowledgeConfig(I2C2,DISABLE);
	    while (!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED));

		*Data=I2C_ReceiveData(I2C2);

		//DMA-s olvasás, jó lenne, ha menne
//	    I2C_DMAReceive(Data,len);
//	    I2C_DMALastTransferCmd(I2C2,ENABLE);
//	    I2C_DMACmd(I2C2,ENABLE);
//	    DMA_Cmd(DMA1_Channel5, ENABLE);
//	    I2C_ITConfig(I2C2,I2C_IT_EVT,ENABLE);
//        while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));

	//    xSemaphoreTake(I2C_TransferComplete,portMAX_DELAY);

//	    I2C_DMACmd(I2C2,DISABLE);
//	    DMA_Cmd(DMA1_Channel5, DISABLE);
//      DMA_ClearFlag(DMA1_FLAG_TC5);

	    I2C_GenerateSTOP(I2C2, ENABLE);
	 	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF)); // stop bit flag

	 	xSemaphoreGive(I2CBusy);
	    return 1;
	}
	return 0;
	*/
}
uint8_t I2C_ByteWrite(uint8_t Data, uint8_t deviceAddress, uint8_t WriteAddr)
{
	if (xSemaphoreTake(I2CBusy,portMAX_DELAY))
	{

	    while (I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));
	    I2C_AcknowledgeConfig(I2C2, ENABLE);

		I2C_GenerateSTART(I2C2, ENABLE);
	    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	    I2C_Send7bitAddress(I2C2, deviceAddress, I2C_Direction_Transmitter);
	    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	    I2C_SendData(I2C2, WriteAddr);
	    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	    I2C_SendData(I2C2, Data);
	    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	    I2C_GenerateSTOP(I2C2, ENABLE);
	    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));

		xSemaphoreGive(I2CBusy);
	    return 1;
	}
	return 0;
}
