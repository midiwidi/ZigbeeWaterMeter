/*
 * Zigbee Water Meter - An open-source Zigbee water meter project.
 * Copyright (c) 2025 Ignacio Hernández-Ros and Markus Wiedemann.
 *
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. To view a copy of this license, visit
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * You may use, modify, and share this work for personal and non-commercial purposes, as long
 * as you credit the original author(s) and share any derivatives under the same license.
 */
#include "esp_zigbee_type.h"
#include "freertos/FreeRTOS.h"

extern const char *TAG;

// output - pin to enable battery voltage to the adc converter
//#define BAT_MON_ENABLE                  		GPIO_NUM_21

// Report event group events
#define CURRENT_SUMMATION_DELIVERED_REPORT  (1 << 0)
#define INSTANTANEOUS_DEMAND_REPORT         (1 << 1)
#define STATUS_REPORT                       (1 << 2)
#define EXTENDED_STATUS_REPORT              (1 << 3)
#define BATTER_REPORT                       (1 << 4)

// Main group events
#define SHALL_MEASURE_BATTERY           (1 << 0)
#define SHALL_ENABLE_ZIGBEE             (1 << 1)
// this is not implemented because of lack of support from esp-zigbee-sdk
// see https://github.com/espressif/esp-zigbee-sdk/issues/561
#define SHALL_DISABLE_ZIGBEE            (1 << 2) 
// Command to stop the deep sleep functionality at all. This is required
// while OTA is updating the firmware. New events shall no reschedule the
// deep sleep timer
#define SHALL_STOP_DEEP_SLEEP						(1 << 3)

// Command to start the deel sleep functionality. When the main loop starts
// and when the OTA is cancelled
#define SHALL_START_DEEP_SLEEP					(1 << 4)

extern EventGroupHandle_t report_event_group_handle;
extern EventGroupHandle_t main_event_group_handle;

extern QueueHandle_t deep_sleep_queue_handle;
extern TaskHandle_t deep_sleep_task_handle;


TickType_t dm_deep_sleep_time_ms();
void gm_counter_set(const esp_zb_uint48_t *new_value);