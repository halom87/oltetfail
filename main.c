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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bt_control_cc256x.h"

#include <btstack/hci_cmds.h>
#include <btstack/run_loop.h>
#include <btstack/sdp_util.h>

#include "hci.h"
#include "l2cap.h"
#include "btstack_memory.h"
#include "remote_device_db.h"
#include "rfcomm.h"
#include "sdp.h"
#include "config.h"


extern __IO uint16_t ADCValue[2];
volatile uint16_t eredmeny,kezdet, vege;
#define TASK_LED_PRIORITY ( tskIDLE_PRIORITY + 1  )
#define TASK_PWMSET_PRIORITY ( tskIDLE_PRIORITY  + 1 )
#define TASK_BTCOMM_PRIORITY ( tskIDLE_PRIORITY  + 3 )
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
// BT kommunikáció teszt
static void prvBTCommTask (void* pvParameters);
//itt képne kiolvasni a szenzorokat
static void prvSensorReadTask (void* pvParameters);

xSemaphoreHandle xADCSemaphore = NULL;
//Ez itt nem fog kelleni, át kell állítani az új bluetooth initjére

xTaskHandle initTask;
xTaskHandle ledTask;
xTaskHandle BTTask;

int main(void)
{
	//uint16_t kezdet, vege;
	vSemaphoreCreateBinary(xADCSemaphore);
	RCC_Config();

	IO_Config();
	PWM_Config();
	DMA_Config();
	I2C_Config();
	NVIC_Config();

	DebugTimerInit();
	xTaskCreate(prvInitTask,(signed char*)"INIT", configMINIMAL_STACK_SIZE,NULL,TASK_INIT_PRIORITY,&initTask);

	vTaskStartScheduler();
  while (1)
  {



  }
}
static void prvInitTask(void* pvParameters)
{
	portBASE_TYPE res = 0;

	//inicializálás
	initSensorACC();
	initSensorGyro();

	//taszk indítás
	res = xTaskCreate(prvLEDTask,(signed char*)"LED", configMINIMAL_STACK_SIZE,NULL,TASK_LED_PRIORITY,&ledTask);
	res = xTaskCreate(prvBTCommTask,(signed char*)"BTComm", configMINIMAL_STACK_SIZE,NULL,TASK_BTCOMM_PRIORITY,&BTTask);
	//xTaskCreate(prvPWMSetTask,(signed char*)"PWM Set", configMINIMAL_STACK_SIZE,NULL,TASK_PWMSET_PRIORITY,NULL);
	//xTaskCreate(prvSensorReadTask,(signed char*)"Sensor Read", configMINIMAL_STACK_SIZE,NULL,TASK_SENSORREAD_PRIORITY,NULL);

	vTaskDelete(initTask);
	while(1)
	{

	}
}

#define HEARTBEAT_PERIOD_MS 1000

static uint8_t   rfcomm_channel_nr = 1;
static uint16_t  rfcomm_channel_id;
static uint8_t   spp_service_buffer[100];

// Bluetooth logic
static void packet_handler (void * connection, uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint16_t  mtu;

	switch (packet_type) {
		case HCI_EVENT_PACKET:
			switch (packet[0]) {

				case BTSTACK_EVENT_STATE:
					// bt stack activated, get started - set local name
					if (packet[2] == HCI_STATE_WORKING) {
                        hci_send_cmd(&hci_write_local_name, "Kvadrokopter");
					}
					break;

				case HCI_EVENT_COMMAND_COMPLETE:
					if (COMMAND_COMPLETE_EVENT(packet, hci_read_bd_addr)){
                        bt_flip_addr(event_addr, &packet[6]);
                        //printf("BD-ADDR: %s\n\r", bd_addr_to_str(event_addr));
                        break;
                    }
					if (COMMAND_COMPLETE_EVENT(packet, hci_write_local_name)){
                        hci_discoverable_control(1);
                        break;
                    }
                    break;

				case HCI_EVENT_LINK_KEY_REQUEST:
					// deny link key request
                    //printf("Link key request\n\r");
                    bt_flip_addr(event_addr, &packet[2]);
					hci_send_cmd(&hci_link_key_request_negative_reply, &event_addr);
					break;

				case HCI_EVENT_PIN_CODE_REQUEST:
					// inform about pin code request
                    //printf("Pin code request - using '0000'\n\r");
                    bt_flip_addr(event_addr, &packet[2]);
					hci_send_cmd(&hci_pin_code_request_reply, &event_addr, 4, "0000");
					break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
					// data: event (8), len(8), address(48), channel (8), rfcomm_cid (16)
					bt_flip_addr(event_addr, &packet[2]);
					rfcomm_channel_nr = packet[8];
					rfcomm_channel_id = READ_BT_16(packet, 9);
					//printf("RFCOMM channel %u requested for %s\n\r", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection_internal(rfcomm_channel_id);
					break;

				case RFCOMM_EVENT_OPEN_CHANNEL_COMPLETE:
					// data: event(8), len(8), status (8), address (48), server channel(8), rfcomm_cid(16), max frame size(16)
					if (packet[2]) {
						//printf("RFCOMM channel open failed, status %u\n\r", packet[2]);
					} else {
						rfcomm_channel_id = READ_BT_16(packet, 12);
						mtu = READ_BT_16(packet, 14);
						//printf("\n\rRFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n\r", rfcomm_channel_id, mtu);
					}
					break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    rfcomm_channel_id = 0;
                    break;

                default:
                    break;
			}
            break;

        default:
            break;
	}
}

static void  heartbeat_handler(struct timer *ts){
	/*
    if (rfcomm_channel_id){
        static int counter = 0;
        char lineBuffer[30];
        //sprintf(lineBuffer, "BTstack counter %04u\n\r", ++counter);
        //printf(lineBuffer);
        int err = rfcomm_send_internal(rfcomm_channel_id, (uint8_t*) lineBuffer, strlen(lineBuffer));
        if (err) {
            //printf("rfcomm_send_internal -> error %d", err);
        }
    }
	*/
    run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    run_loop_add_timer(ts);

}

static void prvBTCommTask (void* pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();


	/// GET STARTED with BTstack ///
	btstack_memory_init();
    run_loop_init(RUN_LOOP_EMBEDDED);

    // init HCI
	hci_transport_t    * transport = hci_transport_h4_dma_instance();
	bt_control_t       * control   = bt_control_cc256x_instance();
    hci_uart_config_t  * config    = hci_uart_config_cc256x_instance();
    remote_device_db_t * remote_db = (remote_device_db_t *) &remote_device_db_memory;
	hci_init(transport, config, control, remote_db);

    // use eHCILL
    bt_control_cc256x_enable_ehcill(1);

    // init L2CAP
    l2cap_init();
    l2cap_register_packet_handler(packet_handler);

    // init RFCOMM
    rfcomm_init();
    rfcomm_register_packet_handler(packet_handler);
    rfcomm_register_service_internal(NULL, rfcomm_channel_nr, 100);  // reserved channel, mtu=100

    // init SDP, create record for SPP and register with SDP
    sdp_init();
	memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    service_record_item_t * service_record_item = (service_record_item_t *) spp_service_buffer;
    sdp_create_spp_service( (uint8_t*) &service_record_item->service_record, 1, "SPP Counter");
    sdp_register_service_internal(NULL, service_record_item);

    // set one-shot timer
    timer_source_t heartbeat;
    heartbeat.process = &heartbeat_handler;
    run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    run_loop_add_timer(&heartbeat);

    // ready - enable irq used in h4 task
    __enable_irq();

 	// turn on!
	hci_power_control(HCI_POWER_ON);

    // go!
    run_loop_execute();

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime,10 * portTICKS_PER_MS);
	}

    // happy compiler!
    return;
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

		vTaskDelayUntil(&xLastWakeTime,1000 * portTICKS_PER_MS);
	}
}

static void prvLEDTask (void* pvParameters)
{
	const portCHAR teszt[]="Hello World!\r\n";
	uint8_t i;
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;
	xLastWakeTime=xTaskGetTickCount();
	for( ;; )
	{
		if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1))  //toggle led
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_1); //set to zero
			GPIO_ResetBits(GPIOA, GPIO_Pin_2); //set to zero
		}
		else
		{
			GPIO_SetBits(GPIOA,GPIO_Pin_1); //set to one
			GPIO_SetBits(GPIOA,GPIO_Pin_2); //set to one
		}

		vTaskDelayUntil(&xLastWakeTime,xFrequency * portTICKS_PER_MS);
	}
}
//szenzor kiolvasas
static void prvSensorReadTask (void* pvParameters)
{
	portTickType xLastWakeTime;
	static uint8_t data[6];
	static uint8_t newData;
	AccAxesRaw_t AccAxes;
	MagAxesRaw_t MagAxes;
	AxesRaw_t GyroAxes;
	uint8_t status;
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
			newData|=2;
			GyroAxes.AXIS_X=((int16_t*)data)[0];
			GyroAxes.AXIS_Y=((int16_t*)data)[1];
			GyroAxes.AXIS_Z=((int16_t*)data)[2];
		}
		if (newData==3)
		{
			newData=0;
		}
		vTaskDelayUntil(&xLastWakeTime,100);
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
