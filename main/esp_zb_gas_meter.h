/*
 * Zigbee Gas Meter - An open-source Zigbee gas meter project.
 * Copyright (c) 2025 Ignacio Hernández-Ros.
 *
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. To view a copy of this license, visit
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * You may use, modify, and share this work for personal and non-commercial purposes, as long
 * as you credit the original author(s) and share any derivatives under the same license.
 */
#include "esp_zigbee_type.h"

extern const char *TAG;

// Report event group events
#define CURRENT_SUMMATION_DELIVERED_REPORT  (1 << 0)
#define INSTANTANEOUS_DEMAND_REPORT         (1 << 1)
#define STATUS_REPORT                       (1 << 2)
#define EXTENDED_STATUS_REPORT              (1 << 3)
#define BATTER_REPORT                       (1 << 4)

extern EventGroupHandle_t report_event_group_handle;

extern QueueHandle_t deep_sleep_queue_handle;

TickType_t dm_deep_sleep_time_ms();
void gm_counter_set(const esp_zb_uint48_t *new_value);