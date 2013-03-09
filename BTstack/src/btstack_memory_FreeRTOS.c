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
 *  btstack_memory_FreeRTOS.c
 *
 *  Created by Kundra Laci
 */

#include <FreeRTOS.h>

#include "../config.h"
#include "btstack_memory.h"

#include "hci.h"
#include "l2cap.h"
#include "rfcomm.h"

/*=============================================================================
=============================================================================*/
void * btstack_memory_hci_connection_get(void)
{
    return pvPortMalloc(sizeof(hci_connection_t));
}
void btstack_memory_hci_connection_free(void *hci_connection)
{
	vPortFree(hci_connection);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_l2cap_service_get(void)
{
    return pvPortMalloc(sizeof(l2cap_service_t));
}
void btstack_memory_l2cap_service_free(void *l2cap_service)
{
	vPortFree(l2cap_service);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_l2cap_channel_get(void)
{
	return pvPortMalloc(sizeof(l2cap_channel_t));
}
void btstack_memory_l2cap_channel_free(void *l2cap_channel)
{
	vPortFree(l2cap_channel);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_rfcomm_multiplexer_get(void)
{
	return pvPortMalloc(sizeof(rfcomm_multiplexer_t));
}
void btstack_memory_rfcomm_multiplexer_free(void *rfcomm_multiplexer)
{
	vPortFree(rfcomm_multiplexer);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_rfcomm_service_get(void)
{
	return pvPortMalloc(sizeof(rfcomm_service_t));
}
void btstack_memory_rfcomm_service_free(void *rfcomm_service)
{
	vPortFree(rfcomm_service);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_rfcomm_channel_get(void)
{
	return pvPortMalloc(sizeof(rfcomm_channel_t));
}
void btstack_memory_rfcomm_channel_free(void *rfcomm_channel)
{
	vPortFree(rfcomm_channel);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_db_mem_device_name_get(void)
{
	return pvPortMalloc(sizeof(db_mem_device_name_t));
}
void btstack_memory_db_mem_device_name_free(void *db_mem_device_name)
{
	vPortFree(db_mem_device_name);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_db_mem_device_link_key_get(void)
{
	return pvPortMalloc(sizeof(db_mem_device_link_key_t));
}
void btstack_memory_db_mem_device_link_key_free(void *db_mem_device_link_key)
{
	vPortFree(db_mem_device_link_key);
}

/*=============================================================================
=============================================================================*/
void * btstack_memory_db_mem_service_get(void)
{
	return pvPortMalloc(sizeof(db_mem_service_t));
}
void btstack_memory_db_mem_service_free(void *db_mem_service)
{
	vPortFree(db_mem_service);
}

/*=============================================================================
=============================================================================*/
void btstack_memory_init(void)
{
	// Memory is inited automatically in first malloc, which can happen any time
}
