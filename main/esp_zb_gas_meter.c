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
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_metering.h"
#include "hal/ieee802154_ll.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

// Maximum difference between the internal counter value and last reported counter value
#define COUNTER_REPORT_DIFF             (10)

// time to send the device to deep sleep when Zigbee radio is on
#define TIME_TO_SLEEP_ZIGBEE_ON         (30 * 1000) // milliseconds

// time to send the device to deep sleep when Zigbee radio is off
#define TIME_TO_SLEEP_ZIGBEE_OFF        (500) // milliseconds

// Maximum time to force a device report
#define MUST_SYNC_MINIMUM_TIME          (24 * 60 * 60) // 1 day in seconds

// How long the MAIN BUTTON must be pressed to consider a LONG PRESS
#define LONG_PRESS_TIMEOUT              5 // seconds

/* Zigbee configuration */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */
#define MY_METERING_ENDPOINT            1

#define ESP_MANUFACTURER_NAME           "\x06""MICASA"
#define ESP_MODEL_IDENTIFIER            "\x08""GasMeter" /* Customized model identifier */
#define ESP_DATE_CODE                   "\x08""20250228"
#define ESP_PRODUCT_URL                 "\x2B""https://github.com/IgnacioHR/ZigbeeGasMeter"

#define NVS_NAMESPACE                   "gas_monitor"
#define NVS_KEY                         "counter"

/* ADC Battery measurement */
#define BAT_BUFFER_READ_LEN             256

static const char *TAG = "MICASA_GAS_METER";

// Measure current summation delivered
static esp_zb_uint48_t current_summation_delivered = {
    .low = 0,
    .high = 0
};

static uint8_t alarm_mask = 0x03;
static uint8_t generic_device_type = 0xFF;

// device status and extended status
static uint8_t device_status = 0x0;
static uint64_t device_extended_status = 0x0;

// changed to true when the value of device_status changes
// so value must be reported
static bool status_report = false;

// changed to true when the value of device_extended_status changes
// so value must be reported
static bool extended_status_report = false;

// Measure instantaneous demand as int24
static esp_zb_int24_t instantaneous_demand = {
    .low = 0,
    .high = 0
};

// m³
static uint8_t unit_of_measure = ESP_ZB_ZCL_METERING_UNIT_M3_M3H_BINARY; 

// 1 m³ for every 100 pulses
static esp_zb_uint24_t multiplier = {
    .low = 1,
    .high = 0
};

// 100 pulses is 1 m³
static esp_zb_uint24_t divisor = {
    .low = 100, // test to avoid report configuration issue
    .high = 0
};      

// Formatting current_summation_delivered with 7 digits and 2 decimal places
static uint8_t summation_formatting = ESP_ZB_ZCL_METERING_FORMATTING_SET(true, 7, 2) ;// 0x72;

// Formatting instantaneous demand with 3 decimal places
static uint8_t demand_formatting = ESP_ZB_ZCL_METERING_FORMATTING_SET(true, 2, 3) ;//0x23; 

// Gas metering type
static uint8_t metering_device_type = ESP_ZB_ZCL_METERING_GAS_METERING; 

// value for the manufacturer_code, At this time this is whatever value 
// I've never seen before. I don't know if there is a value for DIY devices
static uint16_t manufacturer_code = 0x8888;

// time for periodic task that check things internally every minute when not
// in deep sleep
// static uint16_t interval = 60;

// value for the ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID attribute
static uint16_t identify_time = 0;

// set to true when the attribute values have changed and this device
// shall update them to parent device
static bool current_summation_delivered_report = false;

// changes from false to true when the counter has been sent to the
// parent device and the new value is stored
// changes from true to false when a new value is set to the counter
static bool current_summation_delivered_sent = false;

// set to true when the attribute values have changed and this device
// shall update them to parent device
static bool instantaneous_demand_report = false;

// set to true when the counter has changed value and the new value
// needs to be stored to nvr
static bool save_counter_nvr = false;

// Last time a pulse was received
static RTC_DATA_ATTR struct timeval last_pulse_time;

// Last time wakeup pin was received
static int64_t last_main_btn_time = 0;

// Last report sent time
static RTC_DATA_ATTR struct timeval last_report_sent_time = {0};
static RTC_DATA_ATTR uint64_t last_summation_sent = 0;

static int64_t last_instant_demand_calculated_time = 0;

// time the device woke up due to interrupt
static struct timeval gpio_time;

// set to true when the device woke up to check if
// the interrupt was received during the following 4 seconds
static bool check_gpio_time = false;

static bool check_main_btn_interrupt_miss = false;

// flag to reset deep sleep timer
static bool restart_deep_sleep = true;

// flag to avoid the device to fail if the user press
// the external button and restart sleep timer cannot 
// be reset
static bool can_restart_sleep = false;

// flag to detect the user pressed main button while in
// deep sleep mode and report is deferred to the moment
// the radio is ready
static bool deferred_main_btn_required_report = false;

// When the main button is pressed a one time task is started
// to detect a long press without having to wait until
// the user releases the button
static bool start_long_press_detector = false;
static bool started_from_deep_sleep = false;
static bool stop_long_press_detector = false;

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
static bool ignore_enter_sleep = false;

// sleep
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static esp_timer_handle_t deep_sleep_timer;

// long press detector
static esp_timer_handle_t longpress_timer;

// Non volatile memory handle
nvs_handle_t my_nvs_handle;

// This flag controls when the Zigbee Radio has to be enabled in the main loop
static bool shall_enable_radio = false;

// This flagg will shut down the zigbee radio when set to true
static bool shall_disable_radio = false;

// This flag indicates if the Zigbee Radio has been enabled and configured
static bool zigbee_enabled = false;

// temp variabled to debug interrupts lost
static bool magnet_debounced = false;
static bool magnet_down = false;
static bool magnet_up = false;

/* Battery related variables */

static adc_channel_t channel = ADC_CHANNEL_3; // GPIO 3
static adc_continuous_handle_t handle = NULL;
static TaskHandle_t s_task_handle;

// last time battery voltage was adquired 
static RTC_DATA_ATTR struct timeval last_battery_measurement_time;

// turned to true when a new measure has to be obtained
static bool shall_measure_battery = false;

static bool measuring_battery = false;

// values to be sent to zigbee
static RTC_DATA_ATTR uint8_t battery_voltage = 84;
static RTC_DATA_ATTR uint8_t battery_percentage = 100*2;

// when set to true the battery percentage will be included 
// in the next report
static bool battery_report = false;

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
void gm_counter_set(esp_zb_uint48_t *new_value) 
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
int time_diff_ms(struct timeval *out_now, const struct timeval *other)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int time_diff = (now.tv_sec  - other->tv_sec ) * 1000 + (now.tv_usec - other->tv_usec) / 1000;
    if (out_now != NULL) {
        *out_now = now;
    }
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
            shall_enable_radio = time_diff_ms(NULL, &last_report_sent_time) / 1000 >= MUST_SYNC_MINIMUM_TIME;
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
            shall_measure_battery = time_diff_ms(NULL, &last_battery_measurement_time) / 1000 >= MUST_SYNC_MINIMUM_TIME;
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

// response to the ESP_ZB_CORE_REPORT_ATTR_CB_ID callback
static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGD(TAG, "Report received from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster);
    return ESP_OK;
}

// response to the ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID callback: TODO check if needed
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGD(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGD(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
                    variable->status, message->info.cluster,
                    variable->attribute.id, variable->attribute.data.type,
                    variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        variable = variable->next;
    }

    return ESP_OK;
}

// response to the ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID callback: TODO check if needed
static esp_err_t zb_write_attr_resp_handler(const esp_zb_zcl_cmd_write_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGD(TAG, "Write attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_write_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGD(TAG, "Write attribute response: status(%d), attribute(0x%x)",
                    variable->status,
                    variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

// response to report config command
static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGD(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
                 variable->status, message->info.cluster, variable->direction, variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

// transfer values from variables to zigbee radio
static esp_zb_zcl_status_t zb_radio_setup_report_values() 
{
    esp_zb_zcl_status_t status = ESP_ZB_ZCL_STATUS_SUCCESS;
    if (current_summation_delivered_report) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &current_summation_delivered, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of current summation delivered: %d", status);
            return status;
        }
    }
    if (instantaneous_demand_report) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID, &instantaneous_demand, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of instantaneous demand: %d", status);
            return status;
        }
    }
    if (battery_report) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percentage, false);
        battery_percentage -= 2; // for testing purposes let's reduce battery_percentage 1% each time it is reported
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of battery percentage: %d", status);
        }
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage, false);
        battery_voltage -= 1; // for testing purposes let's reduce battery_percentage 1% each time it is reported
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of battery voltage: %d", status);
        }
    }
    if (status_report) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_STATUS_ID, &device_status, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of status: %d", status);
        }
    }
    if (extended_status_report) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_EXTENDED_STATUS_ID, &device_extended_status, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of extended status: %d", status);
        }
    }
    return status;
}

// report attributes to radio
static void zb_radio_send_values() 
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = MY_METERING_ENDPOINT;

    if (current_summation_delivered_report) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
    }

    if (instantaneous_demand_report) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
    }

    if (battery_report) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
    }

    if (status_report) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_STATUS_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
    }

    if (extended_status_report) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_EXTENDED_STATUS_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
    }
}

// Attribute handler
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) 
{
    ESP_LOGD(TAG, "In zb_action_handler callback_id=0x%x", callback_id);
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID:
        ret = zb_write_attr_resp_handler((esp_zb_zcl_cmd_write_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: // 0x0000 attribute value set
        esp_zb_zcl_set_attr_value_message_t* setAtrMsg = (esp_zb_zcl_set_attr_value_message_t*)message;
        if (setAtrMsg->info.dst_endpoint == MY_METERING_ENDPOINT && 
            setAtrMsg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_METERING &&
            setAtrMsg->attribute.id == ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID) {
                gm_counter_set(setAtrMsg->attribute.data.value);
            }
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID: // 0x1005
        esp_zb_zcl_cmd_default_resp_message_t *msg = (esp_zb_zcl_cmd_default_resp_message_t*)message;
        ESP_LOGD(TAG, "Default Response: Status: 0x%04x, Command: %d", msg->status_code, msg->resp_to_cmd);
        ESP_LOGD(TAG, "   Src addr: 0x%04x, Dst Addr: 0x%04x, src ep: %d, dst ep: %d, cluster: 0x%04x, profile: 0x%04x",
                    msg->info.src_address.u.short_addr, msg->info.dst_address, msg->info.src_endpoint, msg->info.dst_endpoint,
                    msg->info.cluster, msg->info.profile);
        ESP_LOGD(TAG, "   Resp status: 0x%04x, frame control: 0x%02x, mfg code: 0x%04x, txn: %d, RSSI: %d",
                    msg->info.status, msg->info.header.fc, msg->info.header.manuf_code, msg->info.header.tsn, msg->info.header.rssi);
        current_summation_delivered_sent = true;
        ret = ESP_OK;
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

// initialize zigbee device
static void esp_zb_task(void *pvParameters) 
{
    ESP_LOGD(TAG, "In esp_zb_task");

    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, // End device
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ED_AGING_TIMEOUT, // 64 minutes
            .keep_alive = ED_KEEP_ALIVE,    // 3 seconds
        },
    };
    esp_zb_sleep_enable(true);
    ESP_LOGD(TAG, "esp_zb_init...");
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = 8,
        .power_source = 0x03,
    };

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME
    ));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER
    ));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_ALARM_MASK_ID, &alarm_mask
    ));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_GENERIC_DEVICE_TYPE_ID, &generic_device_type
    ));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, ESP_DATE_CODE
    ));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_URL_ID, ESP_PRODUCT_URL
    ));

    /* identify cluster create with fully customized */
    // esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = identify_time
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    // ESP_ERROR_CHECK(esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, 
    //     ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time
    // ));


    /* power cluster */
    esp_zb_power_config_cluster_cfg_t power_cfg = {
        .main_voltage = 33,
        .main_freq = 0,
        .main_alarm_mask = 0x07,
        .main_voltage_min = 25,
        .main_voltage_max = 33,
        .main_voltage_dwell = 3600
    };

    esp_zb_attribute_list_t *esp_zb_power_cluster = esp_zb_power_config_cluster_create(&power_cfg);
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                                        &battery_voltage));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                        &battery_percentage));

    /* metering cluster */

    // This code can't be used because current_summation_delivered reporting can't be configured
    // esp_zb_metering_cluster_cfg_t metering_cfg = {
    //     .current_summation_delivered = current_summation_delivered,
    //     .status = device_status,
    //     .uint_of_measure = unit_of_measure,
    //     .summation_formatting = summation_formatting,
    //     .metering_device_type = metering_device_type
    // };
    // esp_zb_attribute_list_t *esp_zb_metering_server_cluster = esp_zb_metering_cluster_create(&metering_cfg);

    esp_zb_attribute_list_t *esp_zb_metering_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_METERING);

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_U48,  // Tipo uint48_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                        &current_summation_delivered));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_STATUS_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_8BITMAP,  // Tipo uint8_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                        &device_status));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_U8,  // Tipo uint8_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                        &unit_of_measure));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_U8,  // Tipo uint8_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                        &summation_formatting));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_U8,  // Tipo uint8_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                        &metering_device_type));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_EXTENDED_STATUS_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_64BITMAP,  // Tipo uint64_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                        &device_extended_status));

    // Multiplier attribute
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_MULTIPLIER_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_U24,
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                        &multiplier));

    // Divisor attribute
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_DIVISOR_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_U24,
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                        &divisor));                   

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID,  // ID del atributo instantaneousDemand
                                        ESP_ZB_ZCL_ATTR_TYPE_S24,  // Tipo int24_t con signo
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                        &instantaneous_demand));

    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_metering_server_cluster,
                                        ESP_ZB_ZCL_CLUSTER_ID_METERING,
                                        ESP_ZB_ZCL_ATTR_METERING_DEMAND_FORMATTING_ID,
                                        ESP_ZB_ZCL_ATTR_TYPE_U8,  // Tipo uint8_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                        &demand_formatting));


    /* client identify cluster */
    esp_zb_attribute_list_t *esp_zb_identify_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);

    /* create cluster lists for this endpoint */
    esp_zb_cluster_list_t *esp_zb_meter_cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_meter_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_meter_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_meter_cluster_list, esp_zb_identify_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_metering_cluster(esp_zb_meter_cluster_list, esp_zb_metering_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(esp_zb_meter_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_ep_list_t *esp_zb_ep_meter_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = MY_METERING_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_METER_INTERFACE_DEVICE_ID,
        .app_device_version = 0
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(esp_zb_ep_meter_list, esp_zb_meter_cluster_list, endpoint_config));

    esp_err_t ret = esp_zb_device_register(esp_zb_ep_meter_list);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, , TAG, "Failed to register meter endpoint");

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_zcl_attr_location_info_t current_summation_delivered_location_info = {
        .attr_id = ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .endpoint_id = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };
    ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(current_summation_delivered_location_info));
    esp_zb_zcl_reporting_info_t *current_summation_reporting_info = esp_zb_zcl_find_reporting_info(current_summation_delivered_location_info);
    current_summation_reporting_info->u.send_info.max_interval = (u_int16_t)MUST_SYNC_MINIMUM_TIME;
    current_summation_reporting_info->u.send_info.min_interval = TIME_TO_SLEEP_ZIGBEE_ON / 2;
    esp_zb_uint48_t current_summation_reporting_delta = {
        .low = COUNTER_REPORT_DIFF,
        .high = 0
    };
    current_summation_reporting_info->u.send_info.delta.u48 = current_summation_reporting_delta;
    esp_zb_zcl_update_reporting_info(current_summation_reporting_info);

    esp_zb_zcl_attr_location_info_t instantaneous_demand_location_info = {
        .attr_id = ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .endpoint_id = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };
    ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(instantaneous_demand_location_info));

    esp_zb_zcl_attr_location_info_t  percentage_location_info = {
        .attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .endpoint_id = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };
    ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(percentage_location_info));

    esp_zb_zcl_attr_location_info_t status_location_info = {
        .attr_id = ESP_ZB_ZCL_ATTR_METERING_STATUS_ID,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .endpoint_id = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };
    ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(status_location_info));

    esp_zb_zcl_attr_location_info_t extended_status_location_info = {
        .attr_id = ESP_ZB_ZCL_ATTR_METERING_EXTENDED_STATUS_ID,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .endpoint_id = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };
    ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(extended_status_location_info));

    esp_zb_set_tx_power(IEEE802154_TXPOWER_VALUE_MIN);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_set_secondary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

// top level comissioning callback
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

// function called when the device leaves the zigee network
void leave_callback(esp_zb_zdp_status_t zdo_status, void* args)
{
    ESP_LOGI(TAG, "Leave status 0x%x", zdo_status);
}

// long press detected
static void longpress_cb(void* arg)
{
    ESP_LOGD(TAG, "Button long press detected");
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
static int dm_deep_sleep_time_ms() {
    const int before_deep_sleep_time_sec = zigbee_enabled || shall_enable_radio ? TIME_TO_SLEEP_ZIGBEE_ON : TIME_TO_SLEEP_ZIGBEE_OFF;
    ESP_LOGD(TAG, "Start one-shot timer for %dms to enter the deep sleep", before_deep_sleep_time_sec );
    return before_deep_sleep_time_sec;
}

// start timer to sleep
static void gm_deep_sleep_start()
{
    ESP_ERROR_CHECK(esp_timer_start_once(deep_sleep_timer, dm_deep_sleep_time_ms() * 1000));
}

// restart timer to sleep to a new period
static void gm_deep_sleep_restart()
{
    ESP_ERROR_CHECK(esp_timer_restart(deep_sleep_timer, dm_deep_sleep_time_ms() * 1000));
}

// entry point to restart (or start) the deep sleep timer
// this entry point is save as it checks the time status first
static void gm_deep_sleep_start_or_restart()
{
    if (esp_timer_is_active(deep_sleep_timer))
        gm_deep_sleep_restart();
    else
        gm_deep_sleep_start();
}

// Stops the timer responsible of detecting MAIN BUTTON long press
static void btn_longpress_stop() 
{
    if (esp_timer_is_active(longpress_timer)) {
        ESP_LOGD(TAG, "Stop long press timer");
        ESP_ERROR_CHECK(esp_timer_stop(longpress_timer));
    }
}

// Starts the timer responsible of detecting MAIN BUTTON long press
static void btn_longpress_start() 
{
    int before_longpress_time_msec = LONG_PRESS_TIMEOUT * 1000;
    if (started_from_deep_sleep) {
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
static void gm_compute_instantaneous_demand(const struct timeval *now, const bool save_time)
{
    if (last_pulse_time.tv_sec != 0 || last_pulse_time.tv_usec != 0) {
        int time_diff_ms = (now->tv_sec  - last_pulse_time.tv_sec ) * 1000 + 
                        (now->tv_usec - last_pulse_time.tv_usec) / 1000;
        if (time_diff_ms > 0) {
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
    if (save_time) {
        last_pulse_time.tv_usec = now->tv_usec;
        last_pulse_time.tv_sec = now->tv_sec;
    }
}

static esp_err_t bat_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;

    ESP_LOGD(TAG, "calibration scheme version is Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return ret;
}

static esp_err_t bat_adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGD(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_RETURN_ON_ERROR(adc_cali_delete_scheme_curve_fitting(handle), TAG, "Failed to delete ADC calibration curve");
    return ESP_OK;
}

static bool IRAM_ATTR bat_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void bat_continuous_adc_init(adc_channel_t channel, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = BAT_BUFFER_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = channel & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void adc_task(void *arg)
{
    ESP_LOGD(TAG, "ADC Task Init...");
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;

    bat_continuous_adc_init(channel, &handle);
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = bat_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    ESP_ERROR_CHECK(bat_adc_calibration_init(ADC_UNIT_1, channel, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle));

    uint8_t result[BAT_BUFFER_READ_LEN] = {0};
    memset(result, 0xcc, BAT_BUFFER_READ_LEN);

    /**
     * This is to show you the way to use the ADC continuous mode driver event callback.
     * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
     * However in this example, the data processing (print) is slow, so you barely block here.
     *
     * Without using this event callback (to notify this task), you can still just call
     * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
     */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t ret_num = 0;
    esp_err_t ret = adc_continuous_read(handle, result, BAT_BUFFER_READ_LEN, &ret_num, 0);
    if (ret == ESP_OK) {
        uint64_t total = 0;
        int16_t values = 0;
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
            uint32_t chan_num = p->type2.channel;
            /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
                uint32_t data = p->type2.data;
                total += data;
                values++;
            }
        }
        if (values > 0) {
            uint32_t average = total / values;
            int voltage;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, average, &voltage));
            float bat_voltage = (float)(0.036585366)*voltage; // convert to 2s lipo ranges and mult by 10
            battery_voltage = bat_voltage;
            battery_percentage = (uint8_t)((bat_voltage-(float)(70.0))*(float)(14.28571429));
            battery_report = true;
            ESP_LOGD(TAG, "Raw: %"PRIu32" Calibrated: %"PRId16"mV Bat Voltage: %1.2fv ZB Voltage: %d ZB Percentage: %d", 
                average, voltage, bat_voltage/(float)(10.0), battery_voltage, battery_percentage);
            gettimeofday(&last_battery_measurement_time, NULL);
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "ADC Task ESP_ERR_TIMEOUT");
        //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
        // TODO: indicate in status that battery can't be read
    }

    ESP_ERROR_CHECK(bat_adc_calibration_deinit(adc1_cali_chan0_handle));
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
    measuring_battery = false;
    vTaskDelete(NULL);
}

// update instantaneous demand to 0 when no pulses are received in a minute
static void gm_reset_instantaneous_demand_task() 
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
static void gm_main_loop_task(void *arg)
{
    ESP_LOGD(TAG, "Main Loop Task...");
    while (true) {
        gm_reset_instantaneous_demand_task();
        if (deferred_main_btn_required_report && can_restart_sleep) {
            ESP_LOGD(TAG, "deferred_main_btn_required_report");
            deferred_main_btn_required_report = false;

            shall_enable_radio = true;
            restart_deep_sleep = true;
            start_long_press_detector = true;
            current_summation_delivered_report = true;
            instantaneous_demand_report = true;
            battery_report = true;
            status_report = true;
            extended_status_report = true;
            shall_measure_battery = true;
        }
        check_shall_measure_battery();
        if (shall_measure_battery && !measuring_battery) {
            shall_measure_battery = false;
            measuring_battery = true;
            xTaskCreate(adc_task, "adc", 4096, NULL, tskIDLE_PRIORITY, &s_task_handle);
        }
        if (check_main_btn_interrupt_miss) {
            ESP_LOGD(TAG, "check_main_btn_interrupt_miss");
            check_main_btn_interrupt_miss = false;
            int level = gpio_get_level(PULSE_PIN);
            if (level == 0) {
                stop_long_press_detector = true;
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
            ESP_LOGD(TAG, "start_long_press_detector");
            start_long_press_detector = false;
            btn_longpress_start();
        }
        if (stop_long_press_detector) {
            ESP_LOGD(TAG, "stop_long_press_detector");
            stop_long_press_detector = false;
            btn_longpress_stop();
        }
        if (shall_enable_radio && !zigbee_enabled) {
            ESP_LOGD(TAG, "shall_enable_radio");
            shall_enable_radio = false;
            zigbee_enabled = true;
            esp_zb_platform_config_t config = {
                .radio_config = {
                    .radio_mode = ZB_RADIO_MODE_NATIVE,
                },
                .host_config = {
                    .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
                },
            };
            ESP_ERROR_CHECK(esp_zb_platform_config(&config));
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

// end zigbee values up if there are changes in the measured values
static void gm_main_loop_zigbee_task(void *arg) 
{
    ESP_LOGD(TAG, "Zigbee Loop Task...");
    while (true) {
        // if device was in deep sleep and woke up from PULSE_PIN rising
        // check_gpio_time is set to true
        // note, if PULSE_PIN goes down due to interrupt failing, then
        // it is reset to false during interrupt handling
        if (current_summation_delivered_report || instantaneous_demand_report || battery_report || status_report || extended_status_report) {
            ESP_LOGI(TAG, "Reporting to client Sum=%s, Instant=%s, Bat=%s, Status=%s, Exten=%s", 
                current_summation_delivered_report ? "Yes": "No",
                instantaneous_demand_report ? "Yes": "No",
                battery_report ? "Yes": "No",
                status_report ? "Yes": "No",
                extended_status_report ? "Yes": "No"
            );
            if (esp_zb_lock_acquire(portMAX_DELAY)) {
                esp_zb_zcl_status_t status = zb_radio_setup_report_values();
                if (status == ESP_ZB_ZCL_STATUS_SUCCESS) {
                    zb_radio_send_values();
                }
                esp_zb_lock_release();
                gettimeofday(&last_report_sent_time, NULL);
                last_summation_sent = current_summation_delivered.high;
                last_summation_sent <<= 32;
                last_summation_sent |= current_summation_delivered.low;

            }
            current_summation_delivered_report = false;
            instantaneous_demand_report = false;
            battery_report = false;
            status_report = false;
            extended_status_report = false;

            ignore_enter_sleep = false;
            restart_deep_sleep = true;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// callback to start deep sleep
static void enter_deep_sleep_cb(void* arg)
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

    const int gpio_wakeup_pin_1 = PULSE_PIN;
    const uint64_t gpio_wakeup_pin_mask_1 = 1ULL << gpio_wakeup_pin_1;
    const int gpio_wakeup_pin_2 = MAIN_BTN;
    const uint64_t gpio_wakeup_pin_mask_2 = 1ULL << gpio_wakeup_pin_2;

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
            if ((ext1mask & gpio_wakeup_pin_mask_2) == gpio_wakeup_pin_mask_2) { // wakeup from MAIN_BTN
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
            if ((ext1mask & gpio_wakeup_pin_mask_1) == gpio_wakeup_pin_mask_1) { // wakeup from PULSE_PIN
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
        gpio_wakeup_pin_mask_1 | gpio_wakeup_pin_mask_2, ESP_EXT1_WAKEUP_ANY_HIGH
    ));

    esp_deep_sleep_disable_rom_logging();

    return ESP_OK;
}

// tasks definitions to satisfy reporting requirements
static esp_err_t gm_tasks_init()
{
    return (
        xTaskCreate(gm_main_loop_zigbee_task, "sensor_quick_update", 4096, NULL, tskIDLE_PRIORITY, NULL) == pdPASS
    ) ? ESP_OK : ESP_FAIL;
}

// Signal handler for zigbee radio
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    ESP_LOGD(TAG, "Shall handle signal 0x%x - %s", sig_type, esp_zb_zdo_signal_to_string(sig_type));
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGD(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGD(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGD(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                can_restart_sleep = true;
                ESP_LOGD(TAG, "Device rebooted");
            }
            ESP_LOGD(TAG, "Deferred driver initialization %s", gm_tasks_init() ? "failed" : "successful");
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGD(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGD(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        esp_zb_zdo_signal_leave_params_t *leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params && leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            esp_zb_nvram_erase_at_start(true);                                          // erase previous network information.
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING); // steering a new network.
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        esp_zb_set_node_descriptor_manufacturer_code(manufacturer_code);
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        ESP_LOGD(TAG, "Can sleep");
        break;
    default:
        ESP_LOGD(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

// PULSE_PIN - GPIO Interruption handler
void IRAM_ATTR gpio_pulse_isr_handler(void* arg) 
{
    static struct timeval now;
    if ((last_pulse_time.tv_sec != 0 || last_pulse_time.tv_usec != 0) && time_diff_ms(&now, &last_pulse_time) <= DEBOUNCE_TIMEOUT) {
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
        if (can_restart_sleep) {
            shall_enable_radio = true;
            restart_deep_sleep = true;
            current_summation_delivered_report = true;
            start_long_press_detector = true;
            started_from_deep_sleep = false;
            battery_report = true;
            status_report = true;
            extended_status_report = true;
            shall_measure_battery = true;
        } else {
            deferred_main_btn_required_report = true;
        }
    }
    if (level == 0) {
        // detect long and short time press
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
    uint64_t wakeup_pin = 1ULL << MAIN_BTN;
                                        //      __
    gpio_config_t io_conf = {           // ____|  |_____
        .intr_type = GPIO_INTR_ANYEDGE, //     ^--^- Interrupt both edges
        .mode = GPIO_MODE_INPUT,        // Input pin
        .pin_bit_mask = pulse_pin | wakeup_pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Can't config gpio for PULSE_PIN and MAIN_PIN pins");

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