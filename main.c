/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
//#define HC05_INIT
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "PWM.h"
#include "GPIO.h"
#include "RCC.h"
//#include "ADC.h"
#include "DMA.h"
#include "NVIC.h"
#include "I2C.h"
#include "MadgwickAHRS.h"
#include "lsm303dlhc_driver.h"
#include "l3g4200d_driver.h"


#include "USART.h"

#include <stdio.h>

extern __IO uint16_t ADCValue[2];
extern xQueueHandle TransmitQueue;
volatile uint16_t eredmeny,kezdet, vege;
#define TASK_LED_PRIORITY ( tskIDLE_PRIORITY + 1  )
#define TASK_PWMSET_PRIORITY ( tskIDLE_PRIORITY  + 1 )
#define TASK_ADCREAD_PRIORITY ( tskIDLE_PRIORITY + 2 )
#define TASK_ADCSTART_PRIORITY ( tskIDLE_PRIORITY + 1 )
#define TASK_INIT_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_SENSORREAD_PRIORITY (tskIDLE_PRIORITY + 3)
//LED villogtató, fut-e az oprendszer
static void prvLEDTask (void *pvParameters);
//PWM beállító taszk, egyelõre mind a négy motorra ugyanazt
static void prvPWMSetTask (void* pvParameters);
//Azok az inicializálások futnak itt amihez kellenek már az oprendszer szolgáltatásai:)
static void prvInitTask (void* pvParameters);
//itt képne kiolvasni a szenzorokat
static void prvSensorReadTask (void* pvParameters);

xSemaphoreHandle xADCSemaphore = NULL;
//Ez itt nem fog kelleni, át kell állítani az új bluetooth initjére
#ifdef HC05_INIT
const portCHAR init[]="AT+UART=38400,0,0\r\n";
#endif
extern xQueueHandle TransmitQueue;

int main(void)
{
	//uint16_t kezdet, vege;
	vSemaphoreCreateBinary(xADCSemaphore);
	RCC_Config();

	IO_Config();
	UART_Config();
	PWM_Config();
	DMA_Config();
	I2C_Config();
	NVIC_Config();

	DebugTimerInit();
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	xTaskCreate(prvInitTask,(signed char*)"INIT", configMINIMAL_STACK_SIZE,NULL,TASK_INIT_PRIORITY,NULL);

	vTaskStartScheduler();
  while (1)
  {



  }
}
static void prvInitTask(void* pvParameters)
{
#ifdef HC05_INIT
	uint8_t i;
	for(i=0;init[i]!='\n';i++)
	{
		xQueueSend(TransmitQueue,init+i,( portTickType )0);
	}

	xQueueSend(TransmitQueue,init+i,( portTickType )0);

	UARTStartSend();
#endif
	//inicializálás
	initSensorACC();
	initSensorGyro();

	//taszk indítás
	xTaskCreate(prvLEDTask,(signed char*)"LED", configMINIMAL_STACK_SIZE,NULL,TASK_LED_PRIORITY,NULL);
	xTaskCreate(prvPWMSetTask,(signed char*)"PWM Set", configMINIMAL_STACK_SIZE,NULL,TASK_PWMSET_PRIORITY,NULL);
	xTaskCreate(prvSensorReadTask,(signed char*)"Sensor Read", configMINIMAL_STACK_SIZE,NULL,TASK_SENSORREAD_PRIORITY,NULL);

	vTaskDelete(NULL);
	while(1)
	{

	}

}
//debug gaz valtozo, bluetoothon keresztul valtoztathato
uint8_t throttle=0;
static void prvPWMSetTask (void* pvParameters)
{
	static uint8_t counter=0;
	static uint8_t gaz=1;
	portTickType xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	uint8_t percent=0;
	while (1)
	{
		percent=0;
		if (counter<60) counter++;
		else if ((counter<69
				+6))
		{

			percent=/*0;//*/gaz*10;
			gaz+=1;
			PWM_SetDutyCycle(percent);
			counter++;
		}
		else if (counter<90 )
			{
			counter++;
			}
		//gege szerint ez igy jo lesz
		else PWM_SetDutyCycle(0);

		//bluetooth
/*
		switch (percent/10)
		{
		case 0 :
			xQueueSend(TransmitQueue,'0',( portTickType )0);
			break;
		case 1 :
			xQueueSend(TransmitQueue,'1',( portTickType )0);
			break;
		case 2 :
			xQueueSend(TransmitQueue,'2',( portTickType )0);
			break;
		case 3 :
			xQueueSend(TransmitQueue,'3',( portTickType )0);
			break;
		case 4 :
			xQueueSend(TransmitQueue,'4',( portTickType )0);
			break;
		case 5 :
			xQueueSend(TransmitQueue,'5',( portTickType )0);
			break;
		case 6 :
			xQueueSend(TransmitQueue,'6',( portTickType )0);
			break;
		case 7 :
			xQueueSend(TransmitQueue,'7',( portTickType )0);
			break;
		case 8 :
			xQueueSend(TransmitQueue,'8',( portTickType )0);
			break;
		case 9 :
			xQueueSend(TransmitQueue,'9',( portTickType )0);
			break;
		case 10 :
			xQueueSend(TransmitQueue,'1',( portTickType )0);
			xQueueSend(TransmitQueue,'0',( portTickType )0);
			break;

		}
		xQueueSend(TransmitQueue,' ',( portTickType )0);
		UARTStartSend();
*/
		vTaskDelayUntil(&xLastWakeTime,1000);
	}
}

static void prvLEDTask (void* pvParameters)
{
	const portCHAR teszt[]="Hello World!\r\n";
	uint8_t i;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 5000;
	xLastWakeTime=xTaskGetTickCount();
	for( ;; )
	{
		char c;
		if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1))  //toggle led
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_1); //set to zero
		}
		else
		{
			GPIO_SetBits(GPIOA,GPIO_Pin_1);//set to one
		}

		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}
//szenzor kiolvasas
static void prvSensorReadTask (void* pvParameters)
{
	portTickType xLastWakeTime;
	static uint8_t data[6];
	static uint8_t newData;
	AccAxesRaw_t AccAxes;
	uint8_t status;
	int16_t x_acc,y_acc,z_acc;
	int16_t x_mag,y_mag,z_mag;
	int16_t x_gyr,y_gyr,z_gyr;
	xLastWakeTime=xTaskGetTickCount();
	while(1)
	{
		//400Hz-el jon az adat a gyorsulas merobol es a gyrobol.
		GetSatusReg(&status);
		if (status&0b0001000) //new data received
		{
			newData|=1;
		//	GetAccAxesRaw(&AccAxes);
			readACC(data);

			//allithato endiannes miatt olvasható így is
			x_acc=((int16_t*)data)[0];
			y_acc=((int16_t*)data)[1];
			z_acc=((int16_t*)data)[2];

		}

		//az iranytubol csak 30Hz
		//ReadStatusM(&status);
		if (status&0x01)
		{
			readMag(data);
			x_mag=((int16_t)data[0])<<8;
			x_mag|=data[1];
			y_mag=((int16_t)data[2])<<8;
			y_mag|=data[3];
			z_mag=((int16_t)data[4])<<8;
			z_mag|=data[5];
		}
		L3G4200D_GetSatusReg(&status);
		if (status&0b00001000)
		{
			ReadGyro(data);
			newData|=2;
			x_gyr=((int16_t*)data)[0];
			y_gyr=((int16_t*)data)[1];
			z_gyr=((int16_t*)data)[2];
		}
		if (newData==3)
		{
			newData=0;
		}
		vTaskDelayUntil(&xLastWakeTime,200);
		float x=2;
		x=invSqrt(x);
		//x?=0.707168
	}

}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
  GPIO_SetBits(GPIOC,GPIO_Pin_12);
  for( ;; );
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
