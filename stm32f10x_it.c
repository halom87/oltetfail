/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "USART.h"

extern xSemaphoreHandle xADCSemaphore;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
/*void SVC_Handler(void)
{
}
*/
/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*void PendSV_Handler(void)
{
}
*/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*void SysTick_Handler(void)
{
}
*/
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

extern xSemaphoreHandle I2C_TransferComplete;

void DMA1_Channel4_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(I2C_TransferComplete,&xHigherPriorityTaskWoken);

	DMA_ClearITPendingBit(DMA1_IT_TC4);
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,DISABLE);


	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );



}
void DMA1_Channel5_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(I2C_TransferComplete,&xHigherPriorityTaskWoken);

	DMA_ClearITPendingBit(DMA1_IT_TC5);
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,DISABLE);

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );



}
void ADC1_2_IRQHandler(void)
{
/*
	static uint8_t i=0;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xADCSemaphore,&xHigherPriorityTaskWoken);
	 ADCValue[i] = ADC_GetConversionValue(ADC1);
	if (i==1) i=0;
	else i=1;
	 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
*/
}
extern xQueueHandle TransmitQueue;
extern uint8_t throttle;
void USART1_IRQHandler (void)
{
	portCHAR cChar;
	uint16_t receivedata;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
	{
		if( xQueueReceiveFromISR( TransmitQueue, &cChar, &xHigherPriorityTaskWoken ) == pdTRUE )
			USART_SendData(USART1,cChar);
		else
		{

			USART_ITConfig( USART1, USART_IT_TXE, DISABLE );
		}
	}
	if (USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		receivedata=USART_ReceiveData(USART1);
		cChar=receivedata&0xFF;
		if (cChar=='w') if (throttle<90) throttle +=10;
		if (cChar=='s') if (throttle>10) throttle -=10;

	}
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



void I2C2_EV_IRQHandler (void)
{
	uint32_t event;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
/*
	if(I2C_GetITStatus(I2C2,I2C_IT_SB)==SET)
	{
		I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
		xSemaphoreGiveFromISR(I2C_SB,&xHigherPriorityTaskWoken);
	}
	if(I2C_GetITStatus(I2C2,I2C_IT_ADDR)==SET)
	{
		I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
		xSemaphoreGiveFromISR(I2C_ADDR,&xHigherPriorityTaskWoken);
	}
	if(I2C_GetITStatus(I2C2,I2C_IT_BTF)==SET)
	{
		I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
		xSemaphoreGiveFromISR(I2C_BTF,&xHigherPriorityTaskWoken);
	}

	event=I2C_GetLastEvent(I2C2);
	if (event&I2C_EVENT_MASTER_BYTE_RECEIVED)
	{
		I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
		xSemaphoreGiveFromISR(I2C_RECEIVED,&xHigherPriorityTaskWoken);
	}

*/
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
