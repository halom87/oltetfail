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
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

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


//Rzrkrt az OS béhzi
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
	uint8_t tmp;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(I2C_TransferComplete,&xHigherPriorityTaskWoken);
	if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		tmp=I2C2->DR;
	}

    /* Disable DMA Channel */
    DMA_Cmd(DMA1_Channel5, DISABLE);
    /* Clear the DMA Transfer Complete flag */
    DMA_ClearFlag(DMA1_FLAG_TC5);
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,DISABLE);
	I2C_DMALastTransferCmd(I2C2,DISABLE);
	I2C_DMACmd(I2C2,DISABLE);

    I2C_GenerateSTOP(I2C2,ENABLE);


	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );



}

extern uint8_t I2C_TransmitNReceive;
extern xQueueHandle I2C_SendQueue;
extern xQueueHandle I2C_ReceiveQueue;
extern xSemaphoreHandle I2C_TransferComplete;
void I2C2_EV_IRQHandler (void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t state=0;
	uint32_t SR1,SR2;

	uint8_t data;
	static uint8_t deviceAddress;
	if (I2C_TransmitNReceive==1)
	{
		switch (state)
		{
		case 0:
		case 1:
			break;
		case 2: //reg address sent, data sending
			if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				state++;
				xQueueReceiveFromISR(I2C_SendQueue,&data,&xHigherPriorityTaskWoken);
				I2C_SendData(I2C2, data);
			}
			else state =0xff;
			break;
		case 3: //data sent stop
			if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				state=5;
				I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
				I2C_GenerateSTOP(I2C2,ENABLE);
				xSemaphoreGiveFromISR(I2C_TransferComplete,&xHigherPriorityTaskWoken);
			}
			else state =0xff;
			break;
		default:
			state=0;
			I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
			break;
		}
	}
	else
	{
		switch (state)
		{
		case 0:
		case 1:
			break;
		case 2: //reg address sent, repeat start
			if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				I2C_DMALastTransferCmd(I2C2,ENABLE);
				I2C_DMACmd(I2C2,ENABLE);
				I2C_GenerateSTART(I2C2, ENABLE);
				state++;
				I2C2->DR=0x00; //delete pending interrupts
			}
			else state =0xfc;
			break;
		case 3: //start bit sent address sending for receive
			if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))
			{
				state++;
				I2C_Send7bitAddress(I2C2,deviceAddress , I2C_Direction_Receiver);
			}
			else
			{
				SR1=I2C2->SR1;
				SR2=I2C2->SR2;

				state = 0xFd;

			}
			break;
		case 4: //address sent, reg address sending
			state=5;
//			I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
			if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			{
				state=5;
				I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
			}
			else state =0xfe;
			break;

		}
	}

	switch (state)
	{
	case 0: //start bit sent address sending
		if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))
		{
			state++;
			xQueueReceiveFromISR(I2C_SendQueue,&deviceAddress,&xHigherPriorityTaskWoken);
			I2C_Send7bitAddress(I2C2,deviceAddress , I2C_Direction_Transmitter);
		}
		else state = 0xFF;
		break;
	case 1: //address sent, reg address sending
		if (I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			state++;
			xQueueReceiveFromISR(I2C_SendQueue,&data,&xHigherPriorityTaskWoken);
			I2C_SendData(I2C2, data);
		}
		else state =0xff;
		break;
	case 5:
		state=0;
		break;
	}
	if (state>5)
	{
			I2C_ITConfig(I2C2,I2C_IT_EVT,DISABLE);
			state=0;
		    I2C_GenerateSTOP(I2C2,ENABLE);
	}

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
