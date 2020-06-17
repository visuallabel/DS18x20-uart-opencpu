/**
 * Copyright 2020 Tampere University
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "custom_feature_def.h"
#include "ql_type.h"
#include "ql_stdlib.h"
#include "ql_uart.h"
#include "ril.h"
#include <stdint.h>
#include <stddef.h>
#include "ql_gpio.h"
#include "ql_error.h"
#include "ql_timer.h"
#include "ql_power.h"

#include "onewire.h"
#include "devices/ds18x20.h"
#include "devices/common.h"

char *get_type_by_id(uint8_t id)
{
	switch (id)
	{
	case DS18S20_FAMILY_CODE:
		return "DS18S20";
	case DS18B20_FAMILY_CODE:
		return "DS18B20";
	case DS1822_FAMILY_CODE:
		return "DS1822";
	default:
		return "unknown";
	}
}

#define DEBUG_PORT UART_PORT0
#define DBG_BUF_LEN 512
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT, ...)                                                                                       \
	{                                                                                                                \
		Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);                                                                       \
		Ql_sprintf(DBG_BUFFER, FORMAT, ##__VA_ARGS__);                                                               \
		if (UART_PORT2 == (DEBUG_PORT))                                                                              \
		{                                                                                                            \
			Ql_Debug_Trace(DBG_BUFFER);                                                                              \
		}                                                                                                            \
		else                                                                                                         \
		{                                                                                                            \
			Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8 *)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER))); \
		}                                                                                                            \
	}

#define UART_PORT UART_PORT1

static void uart_debug_callback(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void *customizedPara)
{
	APP_DEBUG("uart debug callback called, status: %d\n", msg);
}

/**
 * List sensors, from https://github.com/dword1511/onewire-over-uart
 */
static void list_sensors(void)
{
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t c = 0, diff = OW_SEARCH_FIRST;
	int16_t temp_dc;

	while (diff != OW_LAST_DEVICE)
	{
		APP_DEBUG("while\n");
		DS18X20_find_sensor(&diff, id);
		if (diff == OW_ERR_PRESENCE)
		{
			APP_DEBUG("All sensors are offline now.\n");
			ow_finit();
			return;
		}
		if (diff == OW_ERR_DATA)
		{
			APP_DEBUG("Bus error.\n");
			ow_finit();
			return;
		}
		APP_DEBUG("Bus %d Device %03u Type 0x%02hx (%s) ID %02hx%02hx%02hx%02hx%02hx%02hx CRC 0x%02hx ", UART_PORT, c, id[0], get_type_by_id(id[0]), id[6], id[5], id[4], id[3], id[2], id[1], id[7]);
		// fflush(stdout);
		c++;

		if (DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL) == DS18X20_OK)
		{
			while (DS18X20_conversion_in_progress() == DS18X20_CONVERTING)
			{
				Ql_Delay_ms(100); /* It will take a while */
			}
			if (DS18X20_read_decicelsius(id, &temp_dc) == DS18X20_OK)
			{
				/* Copied from my MCU code, so no float point */
				APP_DEBUG("TEMP %3d.%01d C\n", temp_dc / 10, temp_dc > 0 ? temp_dc % 10 : -temp_dc % 10);
				continue;
			}
		}

		APP_DEBUG("MEASURE FAILED!\n");
	}
	APP_DEBUG("Sensors listed.\n");
}

static void read_temp(void)
{
	uint8_t id[] = {40, 39, 86, 153, 11, 0, 0, 151}; // use a known sensor id
	int16_t temp_dc;
	ow_reset();
	if (DS18X20_start_meas(DS18X20_POWER_EXTERN, id) == DS18X20_OK)
	{
		APP_DEBUG("start meas\n");
		while (DS18X20_conversion_in_progress() == DS18X20_CONVERTING)
		{
			APP_DEBUG("in progress\n");
			Ql_Delay_ms(50); /* It will take a while */
		}
		if (DS18X20_read_decicelsius(NULL, &temp_dc) == DS18X20_OK)
		{
			APP_DEBUG("%2d.%01d C", temp_dc / 10, temp_dc > 0 ? temp_dc % 10 : -temp_dc % 10);
			return;
		}
	}
}

int count = 0;
static void timer_callback(u32 timerId, void *param)
{
	APP_DEBUG("timer called %d\n", count++);
	if (ow_init(UART_PORT))
	{
		APP_DEBUG("Bus INIT failed. Check COM port.\n");
		return;
	}

	list_sensors();
	//read_temp();
	//ow_reset();

	ow_finit();

	Ql_Timer_Start(TIMER_ID_USER_START + 2, 3000, FALSE);

	APP_DEBUG("timer finished\n");
}

void proc_main_task(s32 taskId)
{
	s32 ret;
	ST_MSG msg;
	Ql_UART_Register(DEBUG_PORT, uart_debug_callback, NULL);
	Ql_UART_Open(DEBUG_PORT, 115200, FC_NONE);
	APP_DEBUG("[APP] Begin\n");

	Ql_SleepDisable();

	Ql_Timer_Register(TIMER_ID_USER_START + 2, timer_callback, NULL);

	Ql_Timer_Start(TIMER_ID_USER_START + 2, 3000, FALSE);

	while (1)
	{
		Ql_OS_GetMessage(&msg);
		switch (msg.message)
		{
		case MSG_ID_RIL_READY:
			Ql_RIL_Initialize();
			APP_DEBUG("[APP] Ril Ready\n");
			break;
		}
	}
}
