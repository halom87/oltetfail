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
#include "main.h"
#include "math.h"
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
//#include "SERIALDEBUG.h"

#include "USART.h"

#include <stdio.h>

extern __IO uint16_t ADCValue[2];
volatile uint16_t eredmeny,kezdet, vege;
SensorData_t SensorData;
xSemaphoreHandle SensorDataMutex;
xQueueHandle DebugSendQueue;

#define TASK_LED_PRIORITY ( tskIDLE_PRIORITY + 1  )
#define TASK_PWMSET_PRIORITY ( tskIDLE_PRIORITY  + 1 )
#define TASK_BTCOMM_PRIORITY ( tskIDLE_PRIORITY  + 1 )
#define TASK_ADCREAD_PRIORITY ( tskIDLE_PRIORITY + 2 )
#define TASK_ADCSTART_PRIORITY ( tskIDLE_PRIORITY + 1 )
#define TASK_INIT_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_AHRS_PRIORITY (tskIDLE_PRIORITY + 2)
#define TASK_SENSORREAD_PRIORITY (tskIDLE_PRIORITY + 0)

//LED villogtató, fut-e az oprendszer
static void prvLEDTask (void *pvParameters);
//PWM beállító taszk, egyelõre mind a négy motorra ugyanazt
static void prvPWMSetTask (void* pvParameters);
//Azok az inicializálások futnak itt amihez kellenek már az oprendszer szolgáltatásai:)
static void prvInitTask (void* pvParameters);
// BT kommunikáció teszt
static void prvBTCommTask (void* pvParameters);
//itt képne kiolvasni a szenzorokat
static void prvSensorReadTask (void* pvParameters);
//AHRS quaternio számoló
static void prvAHRSTask (void* pvParameters);
//debug uzenetkuldo a szabalyozohoz
static void prvDebugSender (void* pvParameters);

//Ez itt nem fog kelleni, át kell állítani az új bluetooth initjére
#ifdef HC05_INIT
const portCHAR init[]="AT+UART=38400,0,0\r\n";
#endif

int main(void)
{
	//uint16_t kezdet, vege;
	RCC_Config();

	IO_Config();
	Debug_Init();
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
	//UART_StartSend("AT+AB DefaultLocalName Pilo\n\r", 0, 0);
	//UART_StartSend("AT+AB LocalName Pilo\n\r", 0, 0);
	//inicializálás
	UART_StartSend("PiloCopter v0.1 commuinication test...\n\r", 0, 0);
	initSensorACC();
	initSensorGyro();
	//SensorMutex
	SensorDataMutex= xSemaphoreCreateMutex();
	xSemaphoreGive(SensorDataMutex);
	//DebugQueue = xQueueCreate( 3, sizeof( double ) );

	//taszk indítás
	xTaskCreate(prvLEDTask,(signed char*)"LED", configMINIMAL_STACK_SIZE,NULL,TASK_LED_PRIORITY,NULL);
	//xTaskCreate(prvBTCommTask,(signed char*)"BT Comm", configMINIMAL_STACK_SIZE,NULL,TASK_BTCOMM_PRIORITY,NULL);
	xTaskCreate(prvPWMSetTask,(signed char*)"PWM Set", configMINIMAL_STACK_SIZE+128,NULL,TASK_PWMSET_PRIORITY,NULL);
	xTaskCreate(prvSensorReadTask,(signed char*)"Sensor Read", configMINIMAL_STACK_SIZE+128,NULL,TASK_SENSORREAD_PRIORITY,NULL);
	//xTaskCreate(prvDebugSender, (signed char*)"Debug",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY,NULL);
	vTaskDelete(NULL);
	while(1)
	{

	}

}
int8_t DebugSendBuffer[64];
static void prvAHRSTask (void* pvParameters)
{
	static SensorData_t SensorDataLocal;
	static uint16_t counter=0;
	static float e[3];
	portTickType xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	while (1)
	{
		if (xSemaphoreTake(SensorDataMutex,1)==pdTRUE)
		{
			SensorDataLocal=SensorData;
			xSemaphoreGive(SensorDataMutex);
		}
		MadgwickAHRSupdate(SensorDataLocal.Gyro.AXIS_X,SensorDataLocal.Gyro.AXIS_Y,SensorDataLocal.Gyro.AXIS_Z,SensorDataLocal.Acc.AXIS_X,SensorDataLocal.Acc.AXIS_Y,SensorDataLocal.Acc.AXIS_Z,SensorDataLocal.Mag.AXIS_X,SensorDataLocal.Mag.AXIS_Y,SensorDataLocal.Mag.AXIS_Z);
		counter++;
		if (counter==1000)
		{
			counter=0;
			quat_2_euler(e);
			e[0]=e[0]*180/M_PI;
			e[1]=e[1]*180/M_PI;
			e[2]=e[2]*180/M_PI;
			sprintf(DebugSendBuffer,"%f.3 %f.3 %f.3 \r\n",e[0],e[1],e[2]);
			UART_StartSend(DebugSendBuffer,0,0);

		}








		vTaskDelayUntil(&xLastWakeTime,1* portTICK_RATE_MS);


	}
}
uint8_t recvTemp[128];
static void prvBTCommTask (void* pvParameters)
{
	portTickType xLastWakeTime;
	uint8_t HCI_Read_Local_Name[3];
	uint8_t recvPos = 0;

	xLastWakeTime=xTaskGetTickCount();

	HCI_Read_Local_Name[0] = 0x00;
	HCI_Read_Local_Name[1] = 0x14;
	HCI_Read_Local_Name[2] = 0x00;

	vTaskDelayUntil(&xLastWakeTime,500);
	UART_StartSend(HCI_Read_Local_Name, 0, 3);

	for(;;)
	{
		recvPos += UART_TryReceive(recvTemp, recvPos, -1, portMAX_DELAY);
		vTaskDelayUntil(&xLastWakeTime,100);
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
	int8_t dBuffer[64];
	while (1)
	{
		uint8_t i;
		static int8_t percent=0;
		char ch=0;
		int8_t buffer[64];
		static uint8_t tmp=0;
		//percent=0;
		/*while (tmp<4)
		{
			percent=10;

			tmp++;
		}
		/**/
		for (i=0;i<UART_ReceiveControlChar(buffer);i++)
		{

			if (buffer[i]=='w')
				percent+=5;
			if (buffer[i]=='s')
				percent-=5;
			if (buffer[i]=='p')	percent=0;
			if (percent<0) percent=0;
			if (percent>100) percent=100;

			//sprintf(DebugSendBuffer,"Throttle %d\r\n", percent);
			//UART_StartSend(DebugSendBuffer,0,0);
		}

		if (percent!=0)
		{
			sprintf(dBuffer,"Throttle %d\r\n", percent);
			UART_StartSend(dBuffer,0,0);
		}

		PWM_SetDutyCycle(percent);

		vTaskDelayUntil(&xLastWakeTime,1000*portTICK_RATE_MS);
	}
}
SensorData_t SensorData;
static void prvLEDTask (void* pvParameters)
{
	int8_t dbg;
	const portCHAR teszt[]="Hello World!\r\n";
	uint8_t i;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 500;
	xLastWakeTime=xTaskGetTickCount();
	for( ;; )
	{
		char c;
		if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_4))  //toggle led
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_4); //set to zero
		}
		else
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_4); //set to one
		}
		//dbg='a';
		//Debug_String_Length( &dbg , 1);
		vTaskDelayUntil(&xLastWakeTime,xFrequency * portTICK_RATE_MS);
	}
}
//szenzor kiolvasas
#define ACC_MAX 2.0
#define MAG_MAX 1.3
#define GYRO_MAX 250.0
static void prvSensorReadTask (void* pvParameters)
{
	portTickType xLastWakeTime;
	static uint8_t data[6];
	static uint8_t newData;
	AccAxesRaw_t AccAxes;
	MagAxesRaw_t MagAxes;
	AxesRaw_t GyroAxes;
	uint8_t status;
	static uint8_t AHRSstart=0;
	xLastWakeTime=xTaskGetTickCount();
	while(1)
	{
		//400Hz-el jon az adat a gyorsulas merobol es a gyrobol.
		GetSatusReg(&status);
		if (status&0b0001000) //new data received
		{
			newData++;
		//	GetAccAxesRaw(&AccAxes);
			readACC(data);

			//allithato endiannes miatt olvasható így is
			AccAxes.AXIS_X=((int16_t*)data)[0];
			AccAxes.AXIS_Y=((int16_t*)data)[1];
			AccAxes.AXIS_Z=((int16_t*)data)[2];

		}

		//az iranytubol csak 30Hz
		//ReadStatusM(&status);
		if (status&0x01)
		{
			readMag(data);
			MagAxes.AXIS_X=((int16_t)data[0])<<8;
			MagAxes.AXIS_X|=data[1];
			MagAxes.AXIS_Y=((int16_t)data[2])<<8;
			MagAxes.AXIS_Y|=data[3];
			MagAxes.AXIS_Z=((int16_t)data[4])<<8;
			MagAxes.AXIS_Z|=data[5];
		}
		L3G4200D_GetSatusReg(&status);
		if (status&0b00001000)
		{
			ReadGyro(data);
			newData++;
			GyroAxes.AXIS_X=((int16_t*)data)[0];
			GyroAxes.AXIS_Y=((int16_t*)data)[1];
			GyroAxes.AXIS_Z=((int16_t*)data)[2];
		}
		if (newData==2)
		{
			if (AHRSstart==0)
			{
				AHRSstart++;
				xTaskCreate(prvAHRSTask,(int8_t*)"AHRS",configMINIMAL_STACK_SIZE+128,NULL,TASK_AHRS_PRIORITY,NULL);
			}
			newData=0;
			if (xSemaphoreTake(SensorDataMutex,portMAX_DELAY)==pdTRUE)
			{
				SensorData.Acc.AXIS_X=ACC_MAX*AccAxes.AXIS_X/INT16_MAX;
				SensorData.Acc.AXIS_Y=ACC_MAX*AccAxes.AXIS_Y/INT16_MAX;
				SensorData.Acc.AXIS_Z=ACC_MAX*AccAxes.AXIS_Z/INT16_MAX;

				SensorData.Mag.AXIS_X=MAG_MAX*MagAxes.AXIS_X/INT16_MAX;
				SensorData.Mag.AXIS_Y=MAG_MAX*MagAxes.AXIS_Y/INT16_MAX;
				SensorData.Mag.AXIS_Z=MAG_MAX*MagAxes.AXIS_Z/INT16_MAX;

				SensorData.Gyro.AXIS_X=GYRO_MAX*GyroAxes.AXIS_X/INT16_MAX*M_PI/180;
				SensorData.Gyro.AXIS_X=GYRO_MAX*GyroAxes.AXIS_X/INT16_MAX*M_PI/180;
				SensorData.Gyro.AXIS_X=GYRO_MAX*GyroAxes.AXIS_X/INT16_MAX*M_PI/180;
				xSemaphoreGive(SensorDataMutex);
			}


		}
		vTaskDelayUntil(&xLastWakeTime,1 * portTICK_RATE_MS);
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
