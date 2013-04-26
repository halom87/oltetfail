/*
 * USART.c
 *
 *  Created on: 2012.12.08.
 *      Author: Adam
 */

#include "USART.h"

xQueueHandle TransmitQueue;
xQueueHandle ReceiveQueue;

void UART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;

	TransmitQueue=xQueueCreate(BT_BUFFER_SIZE,sizeof(portCHAR));
	ReceiveQueue=xQueueCreate(BT_BUFFER_SIZE,sizeof(portCHAR));

	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;

	USART_Init(USART1,&USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	USART_Cmd(USART1, ENABLE);
}

int UART_StartSend(uint8_t * buffer, int startIndex, int length)
{
	int sent = 0;
	if (length==0)
	{
		while (buffer[startIndex+sent]!=0)
		{
			// Is it full?
			if (BT_BUFFER_SIZE - uxQueueMessagesWaiting(TransmitQueue) == 0) break;
			// At least one free space, put in one
			xQueueSend( TransmitQueue, & buffer[startIndex + sent], 0);
			sent++;
		}
		xQueueSend( TransmitQueue, & buffer[startIndex + sent], 0);
	}
	else for (sent=0; sent<length; sent++)
	{
		// Is it full?
		if (BT_BUFFER_SIZE - uxQueueMessagesWaiting(TransmitQueue) == 0) break;
		// At least one free space, put in one
		xQueueSend( TransmitQueue, & buffer[startIndex + sent], 0);
	}

	// Enable sending
	if (sent > 0)
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	// Return sent items count
	return sent;
}

int UART_TryReceive(uint8_t *buffer, int startIndex, int length, portTickType timeoutPerByte)
{
	int recv = 0;
	uint8_t uchar = 0x00;
	if (length < 0)
	{
		length = uxQueueMessagesWaiting(ReceiveQueue);
	}
	for (recv = 0; recv < length; recv++)
	{
		if (xQueueReceive(ReceiveQueue, &uchar, timeoutPerByte) == pdTRUE)
		{
			// Store data from successful read
			buffer[startIndex + recv] = uchar;
		}
		else
		{
			// Timeout
			break;
		}
	}

	return recv;
}

unsigned int UART_GetReceivedBytesCount()
{
	return uxQueueMessagesWaiting(ReceiveQueue);
}

uint8_t UART_ReceiveControlChar(int8_t  * buffer)
{
	int8_t ch,i;
	uint8_t len=UART_GetReceivedBytesCount();
	if (len==0) return 0;
	UART_TryReceive(buffer,0,1,0);
	for (i=1;i<len;i++)
	{
		UART_TryReceive(&ch,0,1,0);
		if (buffer[i-1]!=ch)
			buffer[i]=ch;
		else len--;
	}
	return len;
}
