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
#include "esp_sleep.h"

// values to be sent to zigbee
extern RTC_DATA_ATTR uint8_t battery_voltage;
extern RTC_DATA_ATTR uint8_t battery_percentage;
extern RTC_DATA_ATTR uint32_t battery_alarm_state;
extern uint8_t battery_voltage_min;
extern uint8_t battery_voltage_th1;
