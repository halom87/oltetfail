/*
 * USART.h
 *
 *  Created on: 2012.12.08.
 *      Author: Adam
 */

#ifndef USART_H_
#define USART_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define BT_BUFFER_SIZE (255)

void UART_Config(void);
// returns the count of bytes sent
int UART_StartSend(uint8_t * buffer, int startIndex, int length);
// return the count of bytes received
// if length is negative, it tries to real all available
// timeoutPerByte can be portMAX_DELAY (if INCLUDE_vTaskSuspend is enabled in OS)
int UART_TryReceive(uint8_t *buffer, int startIndex, int length, portTickType timeoutPerByte);
unsigned int UART_GetReceivedBytesCount();
uint8_t UART_ReceiveControlChar(int8_t  * buffer);

#endif /* USART_H_ */
