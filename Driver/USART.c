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

void UART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;

	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	TransmitQueue=xQueueCreate(TRANSMIT_BUFFER_SIZE,sizeof(portCHAR));
}
void inline UARTStartSend(void)
{
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}
