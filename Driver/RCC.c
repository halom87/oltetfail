/*
 * RCC.c
 *
 *  Created on: 2012.11.03.
 *      Author: Adam
 */

#include "RCC.h"
#include "stm32f10x.h"
RCC_ClocksTypeDef RCC_ClockFreq;

void RCC_Config(void)
{


	//jelenleg belso orarol megy a cucc 64MHz-en
	RCC_GetClocksFreq(&RCC_ClockFreq);

	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}

	RCC_ClockSecuritySystemCmd(ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);



	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC
			| RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_USART2,ENABLE);






}
