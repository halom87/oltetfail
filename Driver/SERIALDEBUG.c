/*
 * debug.c
 *
 *		Project: Pilo
 *  	Original Project: LamborjohnnyII
 *      Author: Laci Kundra
 *
 */
#include <stdarg.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f10x.h"

#include "SERIALDEBUG.h"

// Private functions.
void vNum2String( char *s, uint8_t *pPos, uint32_t u32Number, uint8_t u8Base);
void USART2_Init( void );

// Total buffer size for all debug messages.
#define DEBUG_QUEUE_SIZE	512
xQueueHandle xDebugQueue;
xQueueHandle xControlQueue;

volatile uint8_t connected = 1;

void Debug_Init( void )
{
	xDebugQueue = xQueueCreate( DEBUG_QUEUE_SIZE, sizeof( char ) );
	xControlQueue = xQueueCreate( DEBUG_QUEUE_SIZE, sizeof( char ) );

	USART2_Init();
}

// ============================================================================
void USART2_Init( void ) {
    USART_InitTypeDef USART_InitStructure;

    // USARTx configuration
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// USART configuration
	USART_Init(USART2, &USART_InitStructure);
	// Enable USART
	USART_Cmd(USART2, ENABLE);
}

char temp[128];
uint16_t *imtmp;
// ============================================================================
portTASK_FUNCTION( vDebugTask, pvParameters ) {
	char ch;
	portBASE_TYPE xStatus;
	int32_t state = 0;
	char param[7];
	int32_t i;
	float f = 0.0f;

	/* The parameters are not used. */
	( void ) pvParameters;

    USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
	Debug_String( "Debug task started.\r\n");
	bytes=0;

	for(;;) {
		if (state > 1)
		{
			if (uxQueueMessagesWaiting(xControlQueue) < 7) taskYIELD();
			else
			{
				for (i=0; i<7; i++)
				{
					xQueueReceive(xControlQueue, &(param[i]), 1 * portTICKS_PER_MS);
					param[i] -= '0';
				}
				f = 100*param[0] + 10*param[1] + 1*param[2] + 0.1*param[3] + 0.01*param[4] + 0.001*param[5] + 0.0001*param[6];
				Debug_Printf("Received: %f\r\n", f);
				switch (state) {
					case 2:
						// f contains the float
						break;
					case 3:
						// f contains the float
						break;
					case 4:
						// f contains the float
						break;
					default:
						break;
				}
				state = 0;
			}
		}
		else
		{
			// Process the received bytes from UART
			xStatus = xQueueReceive( xControlQueue, &ch, 100 * portTICKS_PER_MS );
			if( xStatus == pdPASS )
			{
				if (state == 0)
				{
					// Handle Debug Console Commands Here.
					switch ( ch ) {
					// list of commands the console debugger responds to.

					case 'p':
						Debug_Printf( "Enter parameter type!\r\n");
						state = 1;
						break;

					// Add general test code here...
					case 't':
						Debug_Printf( "Test response from johnny :)\r\n" );
						break;


					default:
						break;
					}
				}
				else if (state == 1)
				{
					switch ( ch ) {
					case 'p':
						Debug_Printf( "Waiting for propotional parameter...\r\n");
						state = 2;
						break;
					case 'i':
						Debug_Printf( "Waiting for integrator parameter...\r\n");
						state = 3;
						break;
					case 'd':
						Debug_Printf( "Waiting for derivator parameter...\r\n");
						state = 4;
						break;

					default:
						Debug_Printf( "Bad parameter type!\r\n");
						state = 0;
						break;
					}
				}
			}
		}
	}
}

uint16_t uart2_cnt = 0;

void USART2_IRQHandler( void ) {
	char ch;
	portBASE_TYPE xStatus;
	if ( USART_GetITStatus( USART2, USART_IT_RXNE ) == SET )
	{
		connected = 1;
		ch = USART_ReceiveData( USART2 );
		xStatus = xQueueSendFromISR( xControlQueue, &ch, pdFALSE);
		if (xStatus == pdPASS)
		{
			bytes++;
		}
		else
		{
			while (1);
		}
		USART_ClearITPendingBit( USART2, USART_IT_RXNE );
	}

	if( USART_GetFlagStatus( USART2, USART_FLAG_TXE ) == SET )
	{
		if (connected == 1)
		{
			if (uxQueueMessagesWaitingFromISR( xDebugQueue ) > 0)
			{
				xStatus = xQueueReceiveFromISR( xDebugQueue, &ch, pdFALSE );
				if( xStatus == pdPASS )
				{
					USART_SendData( USART2, ch );
					bytes++;
				}
			}
			else
			{
				USART_ITConfig( USART2, USART_IT_TXE, DISABLE );
			}
		}
	}

	++uart2_cnt;
}

// This function copies the the given string into the OS queue.  If the queue
// is full then the rest of the string is ignored.
// ToDo: Ignoring a full queue is not good.
// ============================================================================
void Debug_String_Length( const char *s , int len) {
	portBASE_TYPE xStatus;

	// Once we start coping a string into the queue we don't want to get
	// interrupted.  The copy must be done quickly since interrupts are off!
	taskENTER_CRITICAL();
	while ( len-- > 0 ) {
		xStatus = xQueueSendToBack( xDebugQueue, s++, 0 );
		if ( xStatus == errQUEUE_FULL ) { while(1); break; }
	}
	taskEXIT_CRITICAL();

	if (connected == 1)
	{
		USART_ITConfig( USART2, USART_IT_TXE, ENABLE );
	}
}

// This function copies the the given string into the OS queue.  If the queue
// is full then the rest of the string is ignored.
// ToDo: Ignoring a full queue is not good.
// ============================================================================
void Debug_String( char *s ) {
	portBASE_TYPE xStatus;

	// Once we start coping a string into the queue we don't want to get
	// interrupted.  The copy must be done quickly since interrupts are off!
	taskENTER_CRITICAL();
	while ( *s ) {
		xStatus = xQueueSendToBack( xDebugQueue, s++, 0 );
		if ( xStatus == errQUEUE_FULL ) { while(1); break; }
	}
	taskEXIT_CRITICAL();

	if (connected == 1)
	{
		USART_ITConfig( USART2, USART_IT_TXE, ENABLE );
	}
}

// Simply print to the debug console a string based on the type of reset.
// ============================================================================
void DebugPrintResetType( void ) {

	if ( PWR_GetFlagStatus( PWR_FLAG_WU ) )
		Debug_Printf( "PWR: Wake Up flag\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_SB ) )
		Debug_Printf( "PWR: StandBy flag.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_PVDO ) )
		Debug_Printf( "PWR: PVD Output.\r\n" );

	if ( RCC_GetFlagStatus( RCC_FLAG_PINRST ) )
		Debug_Printf( "RCC: Pin reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PORRST ) )
		Debug_Printf( "RCC: POR/PDR reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_SFTRST ) )
		Debug_Printf( "RCC: Software reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_IWDGRST ) )
		Debug_Printf( "RCC: Independent Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_WWDGRST ) )
		Debug_Printf( "RCC: Window Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_LPWRRST ) )
		Debug_Printf( "RCC: Low Power reset.\r\n" );
}

// DebugPrintf - really trivial implementation, however, it's reentrant!
// ToDo - This needs a rewrite! Add code to check we're not overflowing.
// ============================================================================
void Debug_Printf(const char *fmt, ...) {
	char sTmp[80];	// String build area.  String lives on the stack!
	uint8_t pos=0;
	char *bp = (char *)fmt;
    va_list ap;
    char c;
    char *p;
    int i;
    float f;

    va_start(ap, fmt);

    while ((c = *bp++)) {
        if (c != '%') {
            sTmp[pos++] = c;
            continue;
        }

        switch ((c = *bp++)) {
			// d - decimal value
			case 'd':
				vNum2String( sTmp, &pos, va_arg(ap, uint32_t), 10);
				break;

			// %x - value in hex
			case 'x':
				sTmp[pos++] = '0';
				sTmp[pos++] = 'x';
				vNum2String( sTmp, &pos, va_arg(ap, uint32_t), 16);
				break;

			// %b - binary
			case 'b':
				sTmp[pos++] = '0';
				sTmp[pos++] = 'b';
				vNum2String( sTmp, &pos, va_arg(ap, uint32_t), 2);
				break;

			// %c - character
			case 'c':
				sTmp[pos++] = va_arg(ap, int);
				break;

			// %i - integer
			case 'i':
				i = va_arg(ap, int32_t);
				if(i < 0){
					sTmp[pos++] = '-';
					vNum2String( sTmp, &pos, (~i)+1, 10);
				} else {
					vNum2String( sTmp, &pos, i, 10);
				}
				break;

			// %s - string
			case 's':
				p = va_arg(ap, char *);
				do {
					sTmp[pos++] = *p++;
				} while (*p);
				break;

			// Tricky float magic
			case 'f':
				f = va_arg(ap, double);
				i = ((float)10000.0*f);
				sTmp[pos+7] = i%10 + '0';
				i /= 10;
				sTmp[pos+6] = i%10 + '0';
				i /= 10;
				sTmp[pos+5] = i%10 + '0';
				i /= 10;
				sTmp[pos+4] = i%10 + '0';
				i /= 10;
				sTmp[pos+3] = '.';
				sTmp[pos+2] = i%10 + '0';
				i /= 10;
				sTmp[pos+1] = i%10 + '0';
				i /= 10;
				sTmp[pos+0] = i%10 + '0';
				pos += 8;
				break;

			// %% - output % character
			case '%':
				sTmp[pos++] = '%';
				break;

			// Else, must be something else not handled.
			default:
				sTmp[pos++] = '?';
				break;
        }
    }
    sTmp[pos++] = 0;		// Mark the end of the string.
    Debug_String( sTmp );	// Copy the string into the OS queue.
    return;
}

//
//
void Debug_PrintChars(const char* buf, int len)
{
	Debug_String_Length( buf, len );
}


// Convert a number to a string - used in vDebugPrintf.
// ============================================================================
void vNum2String( char *s, uint8_t *pPos, uint32_t u32Number, uint8_t u8Base) {

    char buf[33];
    char *p = buf + 33;
    uint32_t c, n;

    *--p = '\0';
    do {
        n = u32Number / u8Base;
        c = u32Number - (n * u8Base);
        if (c < 10) {
            *--p = '0' + c;
        } else {
            *--p = 'a' + (c - 10);
        }
        u32Number /= u8Base;
    } while (u32Number != 0);

    while (*p){
    	s[ *pPos ] = *p;
    	*pPos += 1;
        p++;
    }
    return;
}



