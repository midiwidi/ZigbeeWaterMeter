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
#include "sys/time.h"

#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_metering.h"

// Maximum time to force a device report
#define MUST_SYNC_MINIMUM_TIME          (24 * 60 * 60) // 1 day in seconds

// time to send the device to deep sleep when Zigbee radio is on
#define TIME_TO_SLEEP_ZIGBEE_ON         (30 * 1000) // milliseconds

// time to send the device to deep sleep when Zigbee radio is off
#define TIME_TO_SLEEP_ZIGBEE_OFF        (500) // milliseconds

// Maximum difference between the internal counter value and last reported counter value
#define COUNTER_REPORT_DIFF             (10)

// Measure current summation delivered
extern esp_zb_uint48_t current_summation_delivered;

// changes from false to true when the counter has been sent to the
// parent device and the new value is stored
// changes from true to false when a new value is set to the counter
extern bool current_summation_delivered_sent;

// device status and extended status
extern uint8_t device_status;
extern uint64_t device_extended_status;

// set to true when the attribute values have changed and this device
// shall update them to parent device
extern bool current_summation_delivered_report;

// changed to true when the value of device_status changes
// so value must be reported
extern bool status_report;

// changed to true when the value of device_extended_status changes
// so value must be reported
extern bool extended_status_report;

// Measure instantaneous demand as int24
extern esp_zb_int24_t instantaneous_demand;

// set to true when the attribute values have changed and this device
// shall update them to parent device
extern bool instantaneous_demand_report;

// Last report sent time
extern RTC_DATA_ATTR struct timeval last_report_sent_time;
extern RTC_DATA_ATTR uint64_t last_summation_sent;

// set to true when PULSE_PIN raises. Prevents the deep slep
// to enter when the device is in the middle of a clycle count
// NOTE: observations says that the pulse takes between 
// 1 and 2 seconds to go from LOW -> HIGH -> LOW
// it remains LOW for about 8 seconds.
// when the PULSE_PIN falls to LOW the device can sleep safely
// if no activity is detected in 2 minutes.
// but there is a chance, the PULSE_PIN goes HIGH and remains
// HIGH for longer than 2 minutes, in that case if the device
// never enters deep sleep because the exit condition is always
// met (PULSE_PIN is HIGH) so the device enters an infinite restart
// loop that is not desired.
// the solution at this moment is to never enter sleep when
// PULSE_PIN is HIGH, this ensures the interrupt when failing
// is always detected.
// if the device is in deep sleep near the RISE point it might
// finish the LOW->HIGH->LOW cycle before the interrupt is set
// this is detected in the code and the pulse is added automatically
extern bool ignore_enter_sleep;

// flag to reset deep sleep timer
extern bool restart_deep_sleep;

extern bool joined_network;

void esp_zb_task(void *pvParameters);