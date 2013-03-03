/*
 * Copyright (C) 2011 by Matthias Ringwald
 * Customized by Laszlo Kundra (C)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY MATTHIAS RINGWALD AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 *  hal_tick.c
 *
 *  Implementation using 250 ms ticks provided by OS
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include <btstack/hal_tick.h>

static void dummy_handler(void){};
static void (*tick_handler)(void) = &dummy_handler;
static uint32_t btstack_interval_in_ms = 250;

#define TASK_BTTICK_PRIORITY ( tskIDLE_PRIORITY + 4 )

static void prvBTTickTask (void* pvParameters);

void hal_tick_init(void){
	// create tick handler task
	xTaskCreate(prvBTTickTask,(signed char*)"BT Tick", configMINIMAL_STACK_SIZE,NULL,TASK_BTTICK_PRIORITY,NULL);
}

void hal_tick_set_handler(void (*handler)(void)){
	// no clue, what is this?
    if (handler == NULL){
        tick_handler = &dummy_handler;
        return;
    }
    tick_handler = handler;
}

int  hal_tick_get_tick_period_in_ms(void){
	// get tick period
    return btstack_interval_in_ms;
}

portTickType xBTTickLastWakeTime;
static void prvBTTickTask (void* pvParameters)
{
	xBTTickLastWakeTime=xTaskGetTickCount();
	for(;;)
	{
		// call tick handler
		(*tick_handler)();
		// delay interval
		vTaskDelayUntil(&xBTTickLastWakeTime, btstack_interval_in_ms * portTICK_RATE_MS);
	}
}
