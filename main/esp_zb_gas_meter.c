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
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "sys/time.h"
#include "esp_timer.h"

#include "esp_zb_gas_meter.h"
#include "esp_zb_gas_meter_adc.h"
#include "esp_zb_gas_meter_zigbee.h"

/* Experimental, check if we sleepy device can help the design */
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#endif

/* Hardware configuration */

// input - pin with gas magnetic sensor contact
#define PULSE_PIN                       GPIO_NUM_2

// input - pin for the main button
#define MAIN_BTN                        GPIO_NUM_4

// output - pin to enable battery voltage to the adc converter
#define BAT_MON_ENABLE                  GPIO_NUM_6

// input - pin to measure battery voltage
#define BAT_VLTG                        GPIO_NUM_0

// amount of time to ignore a digital input pin interrupt repetition
#define DEBOUNCE_TIMEOUT                50 /* milliseconds */

/* Human interaction and device configuration */

// How long the MAIN BUTTON must be pressed to consider a LONG PRESS
#define LONG_PRESS_TIMEOUT              5 // seconds

#define NVS_NAMESPACE                   "gas_monitor"
#define NVS_KEY                         "counter"

const char *TAG = "MICASA_GAS_METER";
bool battery_report = false;

// set to true when the counter has changed value and the new value
// needs to be stored to nvr
bool save_counter_nvr = false;

// Last time a pulse was received
RTC_DATA_ATTR struct timeval last_pulse_time;

// Last time wakeup pin was received
int64_t last_main_btn_time = 0;

int64_t last_instant_demand_calculated_time = 0;

// time the device woke up due to interrupt
struct timeval gpio_time;

// set to true when the device woke up to check if
// the interrupt was received during the following 4 seconds
bool check_gpio_time = false;

bool check_main_btn_interrupt_miss = false;

// When the main button is pressed a one time task is started
// to detect a long press without having to wait until
// the user releases the button
bool start_long_press_detector = false;
bool started_from_deep_sleep = false;
bool stop_long_press_detector = false;

// sleep
RTC_DATA_ATTR struct timeval sleep_enter_time;
esp_timer_handle_t deep_sleep_timer;

// long press detector
esp_timer_handle_t longpress_timer;

// Non volatile memory handle
nvs_handle_t my_nvs_handle;

// This flag controls when the Zigbee Radio has to be enabled in the main loop
bool shall_enable_radio = false;

// This flagg will shut down the zigbee radio when set to true
bool shall_disable_radio = false;

// This flag indicates if the Zigbee Radio has been enabled and configured
bool zigbee_enabled = false;

// temp variabled to debug interrupts lost
bool magnet_debounced = false;
bool magnet_down = false;
bool magnet_up = false;

// turned to true when a new measure has to be obtained
bool shall_measure_battery = false;

TaskHandle_t s_task_handle;

// Load counter value from NVS
esp_err_t gm_counter_load_nvs() 
{
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_nvs_handle);
    if (err == ESP_OK) {
        uint64_t saved_count = 0;
        err = nvs_get_u64(my_nvs_handle, NVS_KEY, &saved_count);
        if (err == ESP_OK) {
            current_summation_delivered.low =   saved_count & 0x00000000FFFFFFFF;
            current_summation_delivered.high = (saved_count & 0x0000FFFF00000000) >> 32;
            ESP_LOGI(TAG, "Counter loaded: low=%lu high=%u", current_summation_delivered.low, current_summation_delivered.high);
        } else {
            ESP_LOGI(TAG, "Counter not found in memory so starting from 0");
            err = ESP_OK;
        }
    } else {
        device_extended_status |= ESP_ZB_ZCL_METERING_NV_MEMORY_ERROR;
        extended_status_report = true;
        device_status |= ESP_ZB_ZCL_METERING_GAS_CHECK_METER;
        status_report = true;
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
    }
    return err;
}

// Save counter value to NVS
void gm_counter_save_nvs() 
{
    if (save_counter_nvr) {
        uint64_t to_save_count = current_summation_delivered.high;
        to_save_count <<= 32;
        to_save_count |= current_summation_delivered.low;
        esp_err_t err = nvs_set_u64(my_nvs_handle, NVS_KEY, to_save_count);
        if (err == ESP_OK) {
            nvs_commit(my_nvs_handle);
            ESP_LOGI(TAG, "Counter stored: low=%lu high=%d", current_summation_delivered.low, current_summation_delivered.high);
        } else {
            device_extended_status |= ESP_ZB_ZCL_METERING_NV_MEMORY_ERROR;
            extended_status_report = true;
            device_status |= ESP_ZB_ZCL_METERING_GAS_CHECK_METER;
            status_report = true;
            ESP_LOGE(TAG, "Error saving NVS: %s", esp_err_to_name(err));
        }
        save_counter_nvr = false;
    }
}

// Set counter value and save
void gm_counter_set(const esp_zb_uint48_t *new_value) 
{
    current_summation_delivered.low = new_value->low;
    current_summation_delivered.high = new_value->high;
    current_summation_delivered_sent = false;
    save_counter_nvr = true;
    gm_counter_save_nvs();
    ESP_LOGI(TAG, "Counter value set to:low=%lu high=%d", current_summation_delivered.low, current_summation_delivered.high);
}

// Reset counter value to 0 and save
void gm_counter_reset() 
{
    esp_zb_uint48_t zero = {
        .low = 0,
        .high = 0
    };
    gm_counter_set(&zero);
    ESP_LOGI(TAG, "Counter reset");
}

// Helper function to return the time in milliseconds since now and the other timeval
// received as a parameter
int time_diff_ms(const struct timeval *other)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int time_diff = (now.tv_sec  - other->tv_sec ) * 1000 + (now.tv_usec - other->tv_usec) / 1000;
    return time_diff;
}

// Check conditions to enable radio.
// There are 2 possible conditions: 
// - time pass since last report
// - counter increased 10 times
void check_shall_enable_radio() 
{
    if (!shall_enable_radio && !shall_disable_radio && !zigbee_enabled) {
        if (last_report_sent_time.tv_sec == 0 && last_report_sent_time.tv_usec == 0) {
            shall_enable_radio = true;
        } else {
            shall_enable_radio = time_diff_ms(&last_report_sent_time) / 1000 >= MUST_SYNC_MINIMUM_TIME;
        }
        if (!shall_enable_radio && last_summation_sent > 0) {
            uint64_t current_summation_64 = current_summation_delivered.high;
            current_summation_64 <<= 32;
            current_summation_64 |= current_summation_delivered.low;
            shall_enable_radio = current_summation_64 - last_summation_sent >= COUNTER_REPORT_DIFF;
        }
        if (shall_enable_radio) {
            current_summation_delivered_report = true;
        }
    }
}

// Computes if it is needed to measure battery voltage
void check_shall_measure_battery()
{
    if (!shall_measure_battery && !measuring_battery) {
        if (last_battery_measurement_time.tv_sec == 0 && last_battery_measurement_time.tv_usec == 0) {
            shall_measure_battery = true;
        } else {
            shall_measure_battery = time_diff_ms(&last_battery_measurement_time) / 1000 >= MUST_SYNC_MINIMUM_TIME;
        }
        if (shall_measure_battery) {
            // battery voltage is measured with zigbee radio 
            // turned on
            shall_enable_radio = true;
            current_summation_delivered_report = true;
        }
    }
}

// Adds one to current_summation_delivered and nothing else
void gm_counter_increment()
{
    current_summation_delivered.low += 1; // Adds up 1 cent of m³
    if (current_summation_delivered.low == 0) {
        current_summation_delivered.high += 1;
    }
    current_summation_delivered_sent = false;
    current_summation_delivered_report = true;
    save_counter_nvr = true;
}

// function called when the device leaves the zigee network
void leave_callback(esp_zb_zdp_status_t zdo_status, void* args)
{
    ESP_LOGI(TAG, "Leave status 0x%x", zdo_status);
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        joined_network = false;
    }
}

// long press detected
void longpress_cb(void* arg)
{
    ESP_LOGI(TAG, "Button long press detected");
    uint16_t short_address = esp_zb_get_short_address();
    if (short_address != 0xfffe) {
        ESP_LOGD(TAG, "Leaving network");
        esp_zb_zdo_mgmt_leave_req_param_t leave_request = {
            .device_address = {},
            .dst_nwk_addr = 0xFFFF,
            .remove_children = 0,
            .rejoin = 0
        };
        esp_zb_get_long_address(leave_request.device_address);
        esp_zb_zdo_device_leave_req(&leave_request, leave_callback, NULL);
    }
}

// Compute how long to wait for sleep depending on device conditions
int dm_deep_sleep_time_ms() {
    const int before_deep_sleep_time_sec = zigbee_enabled || shall_enable_radio ? TIME_TO_SLEEP_ZIGBEE_ON : TIME_TO_SLEEP_ZIGBEE_OFF;
    ESP_LOGD(TAG, "Start one-shot timer for %dms to enter the deep sleep", before_deep_sleep_time_sec );
    return before_deep_sleep_time_sec;
}

// start timer to sleep
void gm_deep_sleep_start()
{
    ESP_ERROR_CHECK(esp_timer_start_once(deep_sleep_timer, dm_deep_sleep_time_ms() * 1000));
}

// restart timer to sleep to a new period
void gm_deep_sleep_restart()
{
    ESP_ERROR_CHECK(esp_timer_restart(deep_sleep_timer, dm_deep_sleep_time_ms() * 1000));
}

// entry point to restart (or start) the deep sleep timer
// this entry point is save as it checks the time status first
void gm_deep_sleep_start_or_restart()
{
    if (esp_timer_is_active(deep_sleep_timer))
        gm_deep_sleep_restart();
    else
        gm_deep_sleep_start();
}

// Stops the timer responsible of detecting MAIN BUTTON long press
void btn_longpress_stop() 
{
    if (esp_timer_is_active(longpress_timer))
    {
        ESP_LOGD(TAG, "Stop long press timer");
        ESP_ERROR_CHECK(esp_timer_stop(longpress_timer));
    }
}

// Starts the timer responsible of detecting MAIN BUTTON long press
void btn_longpress_start() 
{
    int before_longpress_time_msec = LONG_PRESS_TIMEOUT * 1000;
    if (started_from_deep_sleep)
    {
        before_longpress_time_msec -= 150; // measured time for the device to start
    }
    ESP_LOGD(TAG, "Start long press timer for %dms", before_longpress_time_msec);
    btn_longpress_stop();
    ESP_ERROR_CHECK(esp_timer_start_once(longpress_timer, before_longpress_time_msec * 1000));
}

// After two consecutive values of current_summation_delivered this method
// computes the instantaneous_demand
// NOTE: there is one special task to set instantaneous demand to 0 when no
// values arrive
void gm_compute_instantaneous_demand(const struct timeval *now, const bool save_time)
{
    if (last_pulse_time.tv_sec != 0 || last_pulse_time.tv_usec != 0)
    {
        int time_diff_ms = (now->tv_sec  - last_pulse_time.tv_sec ) * 1000 + 
                        (now->tv_usec - last_pulse_time.tv_usec) / 1000;
        if (time_diff_ms > 0)
        {
            float time_diff_h = time_diff_ms / (1000.0 * 3600.0); // Convert time to hours/100
            int32_t _instantaneous_demand;
            if (time_diff_h > 0) 
            {
                _instantaneous_demand = (int32_t)((1 / time_diff_h) + 0.5); // compute flow in m³/h
            } 
            else
            {
                _instantaneous_demand = 0;
            }
            // check for a real change in the value
            esp_zb_int24_t new_instantaneous_demand;
            new_instantaneous_demand.low = _instantaneous_demand & 0x0000FFFF;
            new_instantaneous_demand.high = (_instantaneous_demand & 0x00FF0000) >> 16;
            if (new_instantaneous_demand.low != instantaneous_demand.low || new_instantaneous_demand.high != instantaneous_demand.high) {
                instantaneous_demand.low = new_instantaneous_demand.low;
                instantaneous_demand.high = new_instantaneous_demand.high;
                instantaneous_demand_report = true;

                last_instant_demand_calculated_time = esp_timer_get_time();
            }
        }
    }
    if (save_time)
    {
        last_pulse_time.tv_usec = now->tv_usec;
        last_pulse_time.tv_sec = now->tv_sec;
    }
}

// update instantaneous demand to 0 when no pulses are received in a minute
void gm_reset_instantaneous_demand() 
{
    int64_t current_time = esp_timer_get_time();
    if (last_instant_demand_calculated_time == 0) {
        last_instant_demand_calculated_time = current_time;
        return;
    }
    int64_t time_diff_ms = (current_time - last_instant_demand_calculated_time) / 1000;
    if (time_diff_ms > (TIME_TO_SLEEP_ZIGBEE_ON - 2)*1000) {
        // I tried sending values to decrease the instantaneous demand curve, but takes
        // too long to reach 0 so the device never enters deep sleep.
        // At a certain moment I'll have to made a decision
        // either, remove instant_demand attribute from cluster because it
        // makes no sense when on batteries or power the device from external
        // source and add a call here to update instantaneous demand
        if (instantaneous_demand.low != 0 || instantaneous_demand.high != 0) {
            instantaneous_demand.low = 0;
            instantaneous_demand.high = 0;
            instantaneous_demand_report = true;

            last_instant_demand_calculated_time = current_time;
        }
    }
}

// device main loop activities, note
// zigbee activities are not part of the
// main loop, just the critical activities
// to maintain the counter
void gm_main_loop_task(void *arg)
{
    ESP_LOGD(TAG, "Main Loop Task...");
    while (true) {
        gm_reset_instantaneous_demand();
        check_shall_measure_battery();
        if (shall_measure_battery && !measuring_battery) {
            shall_measure_battery = false;
            measuring_battery = true;
            xTaskCreate(adc_task, "adc", 4096, NULL, tskIDLE_PRIORITY, &s_task_handle);
        }
        if (check_main_btn_interrupt_miss || start_long_press_detector) {
            ESP_LOGI(TAG, "check_main_btn_interrupt_miss");
            check_main_btn_interrupt_miss = false;
            int level = gpio_get_level(MAIN_BTN);
            if (level == 0) {
                stop_long_press_detector = true;
                start_long_press_detector = false;
                ESP_LOGD(TAG, "check_main_btn_interrupt_miss stop");
            }
        }
        if (check_gpio_time) {
            ESP_LOGD(TAG, "check_gpio_time");
            check_gpio_time = false;
            int level = gpio_get_level(PULSE_PIN);
            // if PULSE_PIN is low AND check_gpio_time is true we 
            // miss the interrupt so count it now
            if (level == 0) {
                gm_counter_increment();
                gm_compute_instantaneous_demand(&gpio_time, true);
                ignore_enter_sleep = false;
                restart_deep_sleep = true;
            } else {
                ignore_enter_sleep = true;
            }
        }
        if (ignore_enter_sleep) {
            int level = gpio_get_level(PULSE_PIN);
            if (level == 0) {
                ignore_enter_sleep = false;
            }
        }
        if (save_counter_nvr) {
            ESP_LOGD(TAG, "save_counter_nvr");
            gm_counter_save_nvs();
            restart_deep_sleep = true;
        }
        check_shall_enable_radio();
        if (restart_deep_sleep) {
            ESP_LOGD(TAG, "restart_deep_sleep");
            restart_deep_sleep = false;
            gm_deep_sleep_start_or_restart();
        }
        if (start_long_press_detector) {
            ESP_LOGI(TAG, "start_long_press_detector");
            start_long_press_detector = false;
            btn_longpress_start();
        }
        if (stop_long_press_detector) {
            ESP_LOGI(TAG, "stop_long_press_detector");
            stop_long_press_detector = false;
            btn_longpress_stop();
        }
        if (shall_enable_radio && !zigbee_enabled) {
            ESP_LOGI(TAG, "shall_enable_radio");
            shall_enable_radio = false;
            zigbee_enabled = true;
            /* Start Zigbee stack task */
            ESP_ERROR_CHECK(xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL) != pdTRUE);
        }
        if (shall_disable_radio) {
            ESP_LOGD(TAG, "shall_disable_radio");
            // TODO:
            shall_disable_radio = false;
            zigbee_enabled = false;
        }
        if (magnet_debounced) {
            ESP_LOGD(TAG, "Magnet debounced");
            magnet_debounced = false;
        }
        if (magnet_down) {
            ESP_LOGD(TAG, "Magnet released");
            magnet_down = false;
        }
        if (magnet_up) {
            ESP_LOGD(TAG, "Magnet pressed");
            magnet_up = false;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// callback to start deep sleep
void enter_deep_sleep_cb(void* arg)
{
    if (ignore_enter_sleep) {
        ESP_LOGI(TAG, "Enter deep sleep ignored");
    } else {
        /* Enter deep sleep */
        ESP_LOGI(TAG, "Enter deep sleep");
        gettimeofday(&sleep_enter_time, NULL);
        esp_deep_sleep_start();
    }
}

// configure deep sleep for the gas meter
esp_err_t gm_deep_sleep_init()
{
    const esp_timer_create_args_t enter_deep_sleep_timer_args = {
            .callback = &enter_deep_sleep_cb,
            .name = "enter-deep-sleep"
    };

    ESP_RETURN_ON_ERROR(esp_timer_create(&enter_deep_sleep_timer_args, &deep_sleep_timer), TAG, "Can't create deep_sleep_timer");

    const esp_timer_create_args_t longpress_timer_args = {
            .callback = &longpress_cb,
            .name = "long-press"
    };

    ESP_RETURN_ON_ERROR(esp_timer_create(&longpress_timer_args, &longpress_timer), TAG, "Can't create longpress_timer");

    const int gpio_pulse_pin = PULSE_PIN;
    const uint64_t gpio_pulse_pin_mask = 1ULL << gpio_pulse_pin;
    const int gpio_mainbtn_pin = MAIN_BTN;
    const uint64_t gpio_mainbtn_pin_mask = 1ULL << gpio_mainbtn_pin;

    // wake-up reason:
    gettimeofday(&gpio_time, NULL);
    int sleep_time_ms = (gpio_time.tv_sec  - sleep_enter_time.tv_sec ) * 1000 + 
                        (gpio_time.tv_usec - sleep_enter_time.tv_usec) / 1000;
    esp_sleep_wakeup_cause_t wake_up_cause = esp_sleep_get_wakeup_cause();
    switch (wake_up_cause) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep and boot: %dms", sleep_time_ms);
            shall_enable_radio = true;
            current_summation_delivered_report = true;
            ignore_enter_sleep = true;
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT1: {
            bool resolved = false;
            uint64_t ext1mask = esp_sleep_get_ext1_wakeup_status();
            if ((ext1mask & gpio_mainbtn_pin_mask) == gpio_mainbtn_pin_mask) { // wakeup from MAIN_BTN
                ESP_LOGI(TAG, "Wake up from MAIN BUTTON. Time spent in deep sleep and boot: %dms", sleep_time_ms);
                start_long_press_detector = true;
                started_from_deep_sleep = true;
                shall_measure_battery = true;
                shall_enable_radio = true;
                current_summation_delivered_report = true;
                ignore_enter_sleep = true;
                check_main_btn_interrupt_miss = true;
                resolved = true;
            }
            if ((ext1mask & gpio_pulse_pin_mask) == gpio_pulse_pin_mask) { // wakeup from PULSE_PIN
                ESP_LOGI(TAG, "Wake up from GAS PULSE. Time spent in deep sleep and boot: %dms", sleep_time_ms);
                check_gpio_time = true;
                ignore_enter_sleep = true;
                resolved = true;
            }
            if (!resolved) {
                ESP_LOGI(TAG, "Wake up from unknown GPIO. Time spent in deep sleep and boot: %dms", sleep_time_ms);
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(TAG, "Not a deep sleep reset");
            shall_enable_radio = true;
            current_summation_delivered_report = true;
            ignore_enter_sleep = true;
            break;
    }
    check_shall_enable_radio();
    check_shall_measure_battery();

    /* Set the methods of how to wake up: */
    /* 1. RTC timer waking-up */
    int report_time_s = (gpio_time.tv_sec  - last_report_sent_time.tv_sec ) + 
                    (gpio_time.tv_usec - last_report_sent_time.tv_usec) / 1000000;

    const uint64_t wakeup_time_sec = MUST_SYNC_MINIMUM_TIME - report_time_s;
    ESP_LOGD(TAG, "Enabling timer wakeup, %llds", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

    /* PULSE_PIN and MAIN_BTN wake up on pull up */

    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(
        gpio_pulse_pin_mask | gpio_mainbtn_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH
    ));

    esp_deep_sleep_disable_rom_logging();

    return ESP_OK;
}

// PULSE_PIN - GPIO Interruption handler
void IRAM_ATTR gpio_pulse_isr_handler(void* arg) 
{
    struct timeval now;
    gettimeofday(&now, NULL);
    if ((last_pulse_time.tv_sec != 0 || last_pulse_time.tv_usec != 0) && time_diff_ms(&last_pulse_time) <= DEBOUNCE_TIMEOUT) {
        magnet_debounced = true;
        return; // DEBOUNCE
    }

    // Increment current_summation_delivery
    // read pin to determine failing or raising edge
    int level = gpio_get_level(PULSE_PIN);
    if (level == 0) {
        magnet_down = true;
        // failing
        gm_counter_increment();
        gm_compute_instantaneous_demand(&now, true);

        ignore_enter_sleep = false;
        check_gpio_time = false; // avoid double count
    } else {
        magnet_up = true;
        // rising, the device while NOT in deep sleep mode. At this moment, if there
        // is no activity in 30 seconds, the device will try to enter deep sleep but
        // it won't success because the pin level is already high so it will wake up
        // inmediatelly. The solution at this moment is to stop the timer interrupt
        // until the failing edge is detected
        ignore_enter_sleep = true;
    }
}

// MAIN_BTN - GPIO Interruption handler
void IRAM_ATTR gpio_btn_isr_handler(void *arg) 
{
    int level = gpio_get_level(MAIN_BTN);
    int64_t current_time = esp_timer_get_time();
    int64_t time_diff_ms = (current_time - last_main_btn_time) / 1000;
    if (last_main_btn_time > 0 && time_diff_ms <= DEBOUNCE_TIMEOUT)
        return; // DEBOUNCE

    if (level == 1) {
        shall_enable_radio = true;
        restart_deep_sleep = true;
        current_summation_delivered_report = true;
        start_long_press_detector = true;
        started_from_deep_sleep = false;
        battery_report = true;
        status_report = true;
        extended_status_report = true;
        shall_measure_battery = true;
    }
    if (level == 0) {
        stop_long_press_detector = true;
        check_main_btn_interrupt_miss = false;
    }

    last_main_btn_time = current_time;
}

// Configure interrupt to catch pulses from magnetic sensor
// NOTE: When the device is in deep sleep internal pull-up and pull-down 
// resistors aren't used so they are disabled.
// In order to save battery I decided to use pull-down because this is
// how the sensor should stay most of the time
esp_err_t gm_gpio_interrup_init() 
{
    uint64_t pulse_pin  = 1ULL << PULSE_PIN;
                                        //      __
    gpio_config_t io_conf_pulse = {     // ____|  |_____
        .intr_type = GPIO_INTR_ANYEDGE, //     ^--^- Interrupt both edges
        .mode = GPIO_MODE_INPUT,        // Input pin
        .pin_bit_mask = pulse_pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf_pulse), TAG, "Can't config gpio for PULSE_PIN and MAIN_PIN pins");

    uint64_t mainbtn_pin = 1ULL << MAIN_BTN;
                                        //      __
    gpio_config_t io_conf_mainbtn = {   // ____|  |_____
        .intr_type = GPIO_INTR_ANYEDGE, //     ^--^- Interrupt both edges
        .mode = GPIO_MODE_INPUT,        // Input pin
        .pin_bit_mask = mainbtn_pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf_mainbtn), TAG, "Can't config gpio for PULSE_PIN and MAIN_PIN pins");

    // Configure and register interrupt service
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED), TAG, "Can't install isr service");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(PULSE_PIN, gpio_pulse_isr_handler, NULL), TAG, "Can't add PULSE_PIN interrupt handler");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(MAIN_BTN, gpio_btn_isr_handler, NULL), TAG, "Can't add MAIN_BTN interrupt handler");

    // Enable voltage measure enable output pin
    uint64_t bat_volt_enable_pin = 1ULL << BAT_MON_ENABLE;
    gpio_config_t voltage_enable_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = bat_volt_enable_pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&voltage_enable_conf), TAG, "Can't config gpio for BAT_MON_ENABLE pin");

    return ESP_OK;
}

// configure power save
esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        // TODO: explore why this causes a problem 
        // When the device enters deep sleep after a 3s period
        // caused by the counter going up one tick
        // the device wakes up on RTC automatically and this
        // is not desired as it will turn on Zigbee radio
        // as it is defined in code

        // .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

// Entry point
void app_main(void) 
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGD(TAG, "\n");
    ESP_LOGI(TAG, "Starting Zigbee GasMeter...");

    ESP_ERROR_CHECK(gm_gpio_interrup_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    ESP_ERROR_CHECK(gm_counter_load_nvs());
    ESP_ERROR_CHECK(gm_deep_sleep_init());

    // start main loop
    ESP_ERROR_CHECK(xTaskCreate(gm_main_loop_task, "gas_meter_main", 8192, NULL, tskIDLE_PRIORITY, NULL) != pdPASS);
}