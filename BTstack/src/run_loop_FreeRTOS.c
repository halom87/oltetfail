/*
 * Copyright (C) 2011 by Matthias Ringwald
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
 *  run_loop_FreeRTOS.c
 *
 *
 *
 *  Created by Kundra Laci
 */

//#include <btstack/port_ucos.h>
#include <btstack/run_loop.h>

#include <btstack/run_loop_FreeRTOS.h>
#include "hci_transport.h"
//#include "serial.h"

static linked_list_t timers;
static linked_list_t data_sources;

xSemaphoreHandle xSemaphore;

/*=============================================================================
=============================================================================*/
void run_loop_add_data_source(data_source_t *ds)
{
    linked_list_add(&data_sources, (linked_item_t *) ds);
}

/*=============================================================================
=============================================================================*/
int run_loop_remove_data_source(data_source_t *ds)
{
	return linked_list_remove(&data_sources, (linked_item_t *) ds);
}

/*=============================================================================
=============================================================================*/
void run_loop_add_timer(timer_source_t *ts)
{
    linked_item_t *it;
    for(it = (linked_item_t *)&timers; it->next; it = it->next)
    {
        if(ts->timeout < ((timer_source_t *)it->next)->timeout)
        {
            break;
        }
    }

    ts->item.next = it->next;
    it->next = (linked_item_t *)ts;
}

/*=============================================================================
=============================================================================*/
void run_loop_set_timer(timer_source_t *ts, uint32_t timeout_in_ms)
{
    unsigned long ticks = portTICKS_PER_MS * timeout_in_ms;
    ts->timeout = xTaskGetTickCount() + ticks;
}

/*=============================================================================
=============================================================================*/
int run_loop_remove_timer(timer_source_t *ts)
{
	return linked_list_remove(&timers, (linked_item_t *) ts);
}

/*=============================================================================
=============================================================================*/
void embedded_trigger(void)
{
	// Give
	portBASE_TYPE temp;
	xSemaphoreGiveFromISR(xSemaphore, &temp);
}

/*=============================================================================
=============================================================================*/
void run_loop_execute(void)
{
	data_source_t *ds;

    for(;;)
    {
    	// block for a specific time if nothing arrives
        if( xSemaphoreTake( xSemaphore, ( portTickType ) (10 * portTICKS_PER_MS) ) == pdTRUE );

        // process data
		data_source_t *next;
		for (ds = (data_source_t *) data_sources; ds != NULL ; ds = next){
			next = (data_source_t *) ds->item.next; // cache pointer to next data_source to allow data source to remove itself
			ds->process(ds);
		}

        // process timers
        while(timers)
        {
            timer_source_t *ts = (timer_source_t *)timers;
            if(ts->timeout > xTaskGetTickCount()) break;
            run_loop_remove_timer(ts);
            ts->process(ts);
        }
    }
}

/*=============================================================================
=============================================================================*/
void run_loop_init(RUN_LOOP_TYPE type)
{
    timers = NULL;
    data_sources = NULL;
    vSemaphoreCreateBinary(xSemaphore);
}

/*=============================================================================
=============================================================================*/
// Sets how many miliseconds has one tick.
uint32_t embedded_ticks_for_ms(uint32_t time_in_ms)
{
	return time_in_ms * portTICKS_PER_MS;
}

// Queries the current time in ticks.
uint32_t embedded_get_ticks(void)
{
	return xTaskGetTickCount();
}
