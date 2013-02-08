/*
 * NVIC.c
 *
 *  Created on: 2012.11.11.
 *      Author: Adam
 */

#include "NVIC.h"
#include "stm32f10x.h"

void NVIC_Config(void)
{

	NVIC_InitTypeDef NVIC_InitStructure;
/*
	NVIC_InitStructure.NVIC_IRQChannel=ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
*//*
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel4_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
*//*
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel5_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
*/
/*
 	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
//*/
//*

	NVIC_InitStructure.NVIC_IRQChannel=I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
//*/



}
