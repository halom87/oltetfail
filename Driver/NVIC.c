/*
 * NVIC.c
 *
 *  Created on: 2012.11.11.
 *      Author: Adam
 */

#include "NVIC.h"
#include "stm32f10x.h"
#include "stm32f10x_exti.h"

void NVIC_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
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
*////*
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel5_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
//*/

 	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);

//*

	NVIC_InitStructure.NVIC_IRQChannel=I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
 	NVIC_Init(&NVIC_InitStructure);
//*/

 	// BT CTS
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);

	/* Configure EXTI11 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI15_10 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;

	NVIC_Init(&NVIC_InitStructure);


}
