/*
 * DMA.h
 *
 *  Created on: 2012.11.10.
 *      Author: Adam
 */

#ifndef DMA_H_
#define DMA_H_
#include "stm32f10x.h"


void DMA_Config(void);
void I2C_DMAReceive(uint8_t* Buffer, uint8_t NmbrOfBytes);
void I2C_DMATransmit(uint8_t* Buffer, uint8_t NmbrOfBytes);





#endif /* DMA_H_ */
