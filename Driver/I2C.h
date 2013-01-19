/*
 * I2C.h
 *
 *  Created on: 2012.12.13.
 *      Author: Adam
 */

#ifndef I2C_H_
#define I2C_H_
#include "stm32f10x.h"
void I2C_Config(void);
uint8_t I2C_ByteWrite(uint8_t Data, uint8_t deviceAddress, uint8_t WriteAddr);
uint8_t I2C_BufferRead(uint8_t* Data, uint8_t deviceAddress, uint8_t ReadAddr, uint8_t len);



#endif /* I2C_H_ */
