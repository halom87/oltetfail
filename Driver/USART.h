/*
 * USART.h
 *
 *  Created on: 2012.12.08.
 *      Author: Adam
 */

#ifndef USART_H_
#define USART_H_

void UART_Config(void);
int UARTStartSend(uint8_t * buffer, int startIndex, int length);

#endif /* USART_H_ */
