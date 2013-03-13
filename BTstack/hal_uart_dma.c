/**
 * @file  hal_bt.c
 ***************************************************************************/
#include <stm32f10x.h>
#include <stdint.h>
#include <btstack/hal_uart_dma.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_exti.h>
#include "FreeRTOS.h"
#include "task.h"

extern void hal_cpu_set_uart_needed_during_sleep(uint8_t enabled);

void dummy_handler(void){};

// rx state
static uint16_t  bytes_to_read = 0;
static uint8_t * rx_buffer_ptr = 0;

// tx state
static uint16_t  bytes_to_write = 0;
static uint8_t * tx_buffer_ptr = 0;

// handlers
static void (*rx_done_handler)(void) = dummy_handler;
static void (*tx_done_handler)(void) = dummy_handler;
static void (*cts_irq_handler)(void) = dummy_handler;

/**
 * @brief  Initializes the serial communications peripheral and GPIO ports
 *         to communicate with the PAN BT ..
 *
 * @param  none
 *
 * @return none
 */
void hal_uart_dma_init(void)
{
	portTickType temp;
	temp = xTaskGetTickCount();
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	// Min 0.5 sec for reset
	vTaskDelayUntil(&temp, 500 * portTICKS_PER_MS);
	// Power up!
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
    // wait for Bluetooth to power up properly (1 sec)
	vTaskDelayUntil(&temp, 1000 * portTICKS_PER_MS);

    hal_uart_dma_set_baud(115200);
}

int hal_uart_dma_set_baud(uint32_t baud){

	USART_InitTypeDef USART_InitStructure;
    int result = 0;

    // Deinit
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    USART_Cmd(USART1, DISABLE);
    USART_DeInit(USART1);

    // Init
    USART_InitStructure.USART_BaudRate=baud;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;

	USART_Init(USART1,&USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	USART_Cmd(USART1, ENABLE);

	return baud;
}

void hal_uart_dma_set_block_received( void (*the_block_handler)(void)){
    rx_done_handler = the_block_handler;
}

void hal_uart_dma_set_block_sent( void (*the_block_handler)(void)){
    tx_done_handler = the_block_handler;
}

void hal_uart_dma_set_csr_irq_handler( void (*the_irq_handler)(void)){
    if (the_irq_handler){
        NVIC_EnableIRQ(EXTI15_10_IRQn);
        cts_irq_handler = the_irq_handler;
        return;
    }

    NVIC_DisableIRQ(EXTI15_10_IRQn);
    cts_irq_handler = dummy_handler;
}

/**********************************************************************/
/**
 * @brief  Disables the serial communications peripheral and clears the GPIO
 *         settings used to communicate with the BT.
 *
 * @param  none
 *
 * @return none
 **************************************************************************/
void hal_uart_dma_shutdown(void) {
	// nothing
}

void hal_uart_dma_send_block(const uint8_t * data, uint16_t len){

    // printf("hal_uart_dma_send_block, size %u\n\r", len);

    // disable TX interrupts
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    tx_buffer_ptr = (uint8_t *) data;
    bytes_to_write = len;

    // enable TX interrupts
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

static inline void hal_uart_dma_enable_rx(void){
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);  // = 0 - RTS low -> ok
}

static inline void hal_uart_dma_disable_rx(void){
	GPIO_SetBits(GPIOA, GPIO_Pin_12);  // = 1 - RTS high -> stop
}

// int used to indicate a request for more new data
void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len){

    // disable RX interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    rx_buffer_ptr = buffer;
    bytes_to_read = len;

    // enable RX interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    hal_uart_dma_enable_rx();     // enable receive
}

void hal_uart_dma_set_sleep(uint8_t sleep){
    //hal_cpu_set_uart_needed_during_sleep(!sleep);
	// do nothing
}

void USART1_IRQHandler (void)
{
    // find reason
	if (USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		if (bytes_to_read == 0) {
			hal_uart_dma_disable_rx();
			USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  // disable RX interrupts
			return;
		}
		*rx_buffer_ptr = USART_ReceiveData(USART1);
		++rx_buffer_ptr;
		--bytes_to_read;
		if (bytes_to_read > 0) {
			return;
		}
		hal_uart_dma_disable_rx();      // = 1 - RTS high -> stop
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); // disable RX interrupts

		(*rx_done_handler)();
	}
	if( USART_GetITStatus( USART1, USART_IT_TXE ) == SET )
	{
		if (bytes_to_write == 0){
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  // disable TX interrupts
			return;
		}
		USART_SendData(USART1, *tx_buffer_ptr);
		++tx_buffer_ptr;
		--bytes_to_write;

		if (bytes_to_write > 0) {
			return;
		}

		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  // disable TX interrupts

		(*tx_done_handler)();
	}
}


// CTS ISR

extern void ehcill_handle(uint8_t action);
#define EHCILL_CTS_SIGNAL      0x034

void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
	(*cts_irq_handler)();

    // Clear the  EXTI line 11 pending bit
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
}
