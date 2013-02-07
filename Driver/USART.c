/*
 * USART.c
 *
 *  Created on: 2012.12.08.
 *      Author: Adam
 */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "USART.h"
#define TRANSMIT_BUFFER_SIZE (63)

xQueueHandle TransmitQueue;
xQueueHandle ReceiveQueue;

void UART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate=115200;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;

	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	TransmitQueue=xQueueCreate(TRANSMIT_BUFFER_SIZE,sizeof(portCHAR));
	ReceiveQueue=xQueueCreate(TRANSMIT_BUFFER_SIZE,sizeof(portCHAR));
}

int UARTStartSend(uint8_t * buffer, int startIndex, int length)
{
	int sent = 0;
	for (sent=0; sent<length; sent++)
	{
		// Is it full?
		if (TRANSMIT_BUFFER_SIZE - uxQueueMessagesWaiting(TransmitQueue) == 0) break;
		// At least one free space, put in one
		xQueueSend( TransmitQueue, & buffer[startIndex + sent], 0);
	}

	// Enable sending
	if (sent > 0)
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	// Return sent items count
	return sent;
}
