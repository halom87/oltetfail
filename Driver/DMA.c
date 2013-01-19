/*
 * DMA.c
 *
 *  Created on: 2012.11.10.
 *      Author: Adam
 */

#include "DMA.h"
#include "stm32f10x.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define I2C2_DR_Address	((uint32_t)0x40005810)
#define I2C_RxChannel DMA1_Channel5
#define I2C_TxChannel DMA1_Channel4

DMA_InitTypeDef DMA_InitStructure;
//DMA config I2C
void DMA_Config(void)
{

	DMA_DeInit(I2C_RxChannel);
	DMA_DeInit(I2C_TxChannel);
	DMA_InitStructure.DMA_PeripheralBaseAddr=I2C2_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)0;
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize=1;
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority=DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(I2C_RxChannel,&DMA_InitStructure);
	DMA_Init(I2C_TxChannel,&DMA_InitStructure);

//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
//
//	DMA_Cmd(DMA1_Channel1, ENABLE);

}
//DMA Fogadás beállítása
void I2C_DMAReceive(uint8_t * Buffer, uint8_t NmbrOfBytes)
{
	DMA_DeInit(I2C_RxChannel);

	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize=(uint32_t)NmbrOfBytes;
	DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)Buffer;
	DMA_Cmd(I2C_RxChannel, DISABLE);

	DMA_Init(I2C_RxChannel,&DMA_InitStructure);
	DMA_ITConfig(I2C_RxChannel,DMA_IT_TC,ENABLE);
	DMA_Cmd(I2C_RxChannel, ENABLE);
}
//DMA küldés beállítása
void I2C_DMATransmit(uint8_t * Buffer, uint8_t NmbrOfBytes)
{
	DMA_DeInit(I2C_TxChannel);

	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize=(uint32_t)NmbrOfBytes;
	DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)Buffer;

	DMA_Init(I2C_TxChannel,&DMA_InitStructure);
	DMA_ITConfig(I2C_TxChannel,DMA_IT_TC,ENABLE);
	DMA_Cmd(I2C_TxChannel, ENABLE);
}
