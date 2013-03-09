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

static volatile linked_list_t timers;
static volatile data_source_t *transportDataSource = 0;

/*=============================================================================
*  Run loop message queue.
*============================================================================*/
#define MSG_QUEUE_BUFFER_SIZE 32

#define MSG_ID_TRIGGER 					   10
#define MSG_ID_INCOMING_TRANSPORT_PACKET   1
#define MSG_ID_OUTGOING_RFCOMM_DATA        2

xQueueHandle messages;

/*=============================================================================
=============================================================================*/
void run_loop_notify_incoming_transport_packet(void)
{
	portBASE_TYPE temp;
	uint8_t id = MSG_ID_INCOMING_TRANSPORT_PACKET;
	xQueueSendFromISR( messages, &id, &temp);
}

/*=============================================================================
=============================================================================*/
void run_loop_notify_outgoing_rfcomm_data(void)
{
	portBASE_TYPE temp;
	uint8_t id = MSG_ID_OUTGOING_RFCOMM_DATA;
	xQueueSendFromISR( messages, &id, &temp);
}

/*=============================================================================
=============================================================================*/
void run_loop_add_data_source(data_source_t *ds)
{
    transportDataSource = ds;
}

/*=============================================================================
=============================================================================*/
int run_loop_remove_data_source(data_source_t *ds)
{
	// Return whatever
    return 0;
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
	portBASE_TYPE temp;
	uint8_t id = MSG_ID_TRIGGER;
	xQueueSendFromISR( messages, &id, &temp);
}

/*=============================================================================
=============================================================================*/
void run_loop_execute(void)
{
    uint8_t event;
    portBASE_TYPE res;

    for(;;)
    {
        res = xQueueReceive(messages, &event, 2);

        if(res == pdTRUE)
        {
            if((unsigned long)event == MSG_ID_INCOMING_TRANSPORT_PACKET)
            {
                transportDataSource->process(transportDataSource);
            }
        }

        /* Process timers. */
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
    timers = 0;

    messages = xQueueCreate( MSG_QUEUE_BUFFER_SIZE, sizeof(uint8_t) );
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
