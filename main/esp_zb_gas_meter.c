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

/* Hardware configuration */
#define PULSE_PIN                       GPIO_NUM_2
#define BUTTON_PIN                      GPIO_NUM_4
#define DEBOUNCE_TIMEOUT                50 /* milliseconds */

/* Zigbee configuration */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */
#define MY_METERING_ENDPOINT            1

#define ESP_MANUFACTURER_NAME           "\x06""MICASA"
#define ESP_MODEL_IDENTIFIER            "\x08""GasMeter" /* Customized model identifier */

#define NVS_NAMESPACE                   "gas_monitor"
#define NVS_KEY                         "counter"

#define SLEEP_WAKEUP_INTERVAL_SEC       (30 * 60) // 0.5 hour (1800 seconds)
#define ACTIVE_TIME_AFTER_INTERRUPT     (2 * 60) // 2 minutes in seconds

#define LONG_PRESS_TIMEOUT              3 // seconds

static const char *TAG = "MICASA_GAS_METER";

// Measure current summation delivered
static esp_zb_uint48_t current_summation_delivered = {
    .low = 0,
    .high = 0
};

// device version
static uint8_t version = 0;

// device status
static uint8_t status = 0x0;

// Measure instantaneous demand as int24
static esp_zb_int24_t instantaneous_demand = {
    .low = 0,
    .high = 0
};

// 0x03 = Battery, 0x04 = DC
static uint8_t power_source = 0x03; 

// m³
static uint8_t unit_of_measure = 0x01; 

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
static uint16_t interval = 60;

// value for the ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID attribute
static uint8_t identify_time = 0;

// set to true when the attribute values have changed and this device
// shall update them to parent device
static bool force_report = false;

// set to true when the counter has changed value and the new value
// needs to be stored to nvr
static bool save_counter_nvr = false;

// Last time a pulse was received
static int64_t last_pulse_time = 0;

// Last time wakeup pin was received
static int64_t last_wakeup_time = 0;

// time the device woke up due to interrupt
static int64_t gpio_time = 0;

// set to true when the device woke up to check if
// the interrupt was received during the following 4 seconds
static bool check_gpio_time = false;

// flag to reset deep sleep timer
static bool restart_deep_sleep = false;

// flag to avoid the device to fail if the user press
// the external button and restart sleep timer cannot 
// be reset
static bool can_restart_sleep = false;

// When the main button is pressed a one time task is started
// to detect a long press without having to wait until
// the user releases the button
static bool start_long_press_detector = false;
static bool started_from_deep_sleep = false;
static bool stop_long_press_detector = false;

// TODO: measure this
// static uint8_t battery_voltage = 29;
// static uint8_t battery_percentage = 95;

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
static RTC_DATA_ATTR struct timeval s_sleep_enter_time;
static esp_timer_handle_t s_oneshot_timer;

// long press detector
static esp_timer_handle_t s_longpress_timer;

// Non volatile memory handle
nvs_handle_t my_nvs_handle;

// Load counter value from NVS
esp_err_t zb_counter_load_nvs() 
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
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
    }
    return err;
}

// Save counter value to NVS
void zb_counter_save_nvs() 
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
            ESP_LOGE(TAG, "Error saving NVS: %s", esp_err_to_name(err));
        }
        save_counter_nvr = false;
    }
}

// Set counter value and save
void zb_counter_set(esp_zb_uint48_t new_value) 
{
    current_summation_delivered.low = new_value.low;
    current_summation_delivered.high = new_value.high;
    save_counter_nvr = true;
    zb_counter_save_nvs();
    ESP_LOGI(TAG, "Counter value set to:low=%lu high=%d", current_summation_delivered.low, current_summation_delivered.high);
}

// Reset counter value to 0 and save
void zb_counter_reset() 
{
    esp_zb_uint48_t zero = {
        .low = 0,
        .high = 0
    };
    zb_counter_set(zero);
    ESP_LOGI(TAG, "Counter reset");
}

// Adds one to current_summation_delivered and nothing else
void zb_counter_increment()
{
    current_summation_delivered.low += 1; // Adds up 1 cent of m³
    if (current_summation_delivered.low == 0) {
        current_summation_delivered.high += 1;
    }
}

// attribute handler for current_summation_delivered to be set via attribute command: TODO check if needed
static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t* attribute)
{
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_METERING) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U48) {
            // current_summation_delivered.low = *(esp_zb_uint48_t *)attribute->data.value.low;
            // current_summation_delivered.high = *(esp_zb_uint48_t *)attribute->data.value.high;
            ESP_LOGI(TAG, "Shall this? Update Current Summation Delivered?: low=%lu high=%d", 
                ((esp_zb_uint48_t *)attribute->data.value)->low, 
                ((esp_zb_uint48_t *)attribute->data.value)->high
            );
        }
    }
}

// response to the ESP_ZB_CORE_REPORT_ATTR_CB_ID callback: TODO check if needed
static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster);
    esp_app_zb_attribute_handler(message->cluster, &message->attribute);
    return ESP_OK;
}

// response to the ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID callback: TODO check if needed
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
                    variable->status, message->info.cluster,
                    variable->attribute.id, variable->attribute.data.type,
                    variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
            esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
        }

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
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
                 variable->status, message->info.cluster, variable->direction, variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

// transfer values from variables to zigbee radio
static void zb_radio_setup_values(bool adquire) {
    if ((adquire && esp_zb_lock_acquire(portMAX_DELAY)) || !adquire) {
        // ESP_LOGI(TAG, "Lock adquired");
        esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &current_summation_delivered, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Status not success updating value of current summation delivered: %d", status);
        }
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID, &instantaneous_demand, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Status not success updating value of instantaneous demand: %d", status);
        }

        // ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd1));
        // ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd2));
        if (adquire)
            esp_zb_lock_release();
    }
}

// report attributes to radio
static void zb_radio_send_values(bool adquire) {
    // this blows up the network anc causes all devices to become offline!
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd1 = {0};
    report_attr_cmd1.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd1.attributeID = ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID;
    report_attr_cmd1.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd1.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
    report_attr_cmd1.zcl_basic_cmd.src_endpoint = MY_METERING_ENDPOINT;

    esp_zb_zcl_report_attr_cmd_t report_attr_cmd2 = {0};
    report_attr_cmd2.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd2.attributeID = ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID;
    report_attr_cmd2.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd2.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
    report_attr_cmd2.zcl_basic_cmd.src_endpoint = MY_METERING_ENDPOINT;

    if ((adquire && esp_zb_lock_acquire(portMAX_DELAY)) || !adquire) {
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd1));
        if (adquire)
            esp_zb_lock_release();
    }
    if ((adquire && esp_zb_lock_acquire(portMAX_DELAY)) || !adquire) {
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd2));
        if (adquire)
            esp_zb_lock_release();
    }
}

// response to attribute action 0x1005
static esp_err_t zb_default_response(const esp_zb_zcl_cmd_default_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    // Does nothing??
    // ESP_LOGI(TAG, "esp_zb_zcl_cmd_default_resp_message_t");
    // ESP_LOGI(TAG, "  info");
    // log_zcl_cmd_info(&message->info, 4);
    // ESP_LOGI(TAG, "  resp_to_cmd: %d", message->resp_to_cmd);
    // ESP_LOGI(TAG, "  status_code: 0x%x", message->status_code);

    // zb_radio_setup_values(false);

    return ESP_OK;
}

// Attribute handler
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    ESP_LOGI(TAG, "In zb_action_handler callback_id=0x%x", callback_id);
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: // 0x0000 attribute value set
        esp_zb_zcl_set_attr_value_message_t* setAtrMsg = (esp_zb_zcl_set_attr_value_message_t*)message;
        if (setAtrMsg->info.dst_endpoint == MY_METERING_ENDPOINT && 
            setAtrMsg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_METERING &&
            setAtrMsg->attribute.id == ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID) {
                esp_zb_uint48_t *value = setAtrMsg->attribute.data.value;
                ESP_LOGI(TAG, "Counter value set: low=%lu high=%d", value->low, value->high);
                current_summation_delivered.high = value->high;
                current_summation_delivered.low = value->low;
                save_counter_nvr = true;
                zb_counter_save_nvs();
            }
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID: // 0x1005
        ret = zb_default_response((esp_zb_zcl_cmd_default_resp_message_t *)message);
        esp_zb_zcl_cmd_default_resp_message_t* msg = (esp_zb_zcl_cmd_default_resp_message_t*)message;
        ESP_LOGI(TAG, "Default response callback: dst 0x%x, status: %u", msg->info.dst_address, msg->status_code);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

// initialize zigbee device
static void esp_zb_task(void *pvParameters) {
    ESP_LOGI(TAG, "In esp_zb_task");

    uint16_t short_address = esp_zb_get_short_address();
    ESP_LOGI(TAG, "Device short address 1 0x%x", short_address);

    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, // End device
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ED_AGING_TIMEOUT, // 64 minutes
            .keep_alive = ED_KEEP_ALIVE,    // 3 seconds
        },
    };
    ESP_LOGI(TAG, "esp_zb_init...");
    esp_zb_init(&zb_nwk_cfg);

    short_address = esp_zb_get_short_address();
    ESP_LOGI(TAG, "Device short address 2 0x%x", short_address);

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = version,
        .power_source = power_source,
    };

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME
    ));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER
    ));

    /* identify cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    ESP_ERROR_CHECK(esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, 
        ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time
    ));

    /* power cluster */
    // esp_zb_attribute_list_t *esp_zb_power_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    // ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_power_cluster,
    //                                     ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
    //                                     ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
    //                                     ESP_ZB_ZCL_ATTR_TYPE_U8,  // Tipo uint8_t
    //                                     ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
    //                                     &battery_voltage));

    // ESP_ERROR_CHECK(esp_zb_cluster_add_attr(esp_zb_power_cluster,
    //                                     ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
    //                                     ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
    //                                     ESP_ZB_ZCL_ATTR_TYPE_U8,  // Tipo uint8_t
    //                                     ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
    //                                     &battery_percentage));

    /* metering cluster */
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
                                        ESP_ZB_ZCL_ATTR_TYPE_U8,  // Tipo uint8_t
                                        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                        &status));

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
    esp_zb_cluster_list_add_basic_cluster(esp_zb_meter_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_meter_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_meter_cluster_list, esp_zb_identify_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_metering_cluster(esp_zb_meter_cluster_list, esp_zb_metering_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // esp_zb_cluster_list_add_power_config_cluster(esp_zb_meter_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_meter_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = MY_METERING_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_METER_INTERFACE_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_meter_list, esp_zb_meter_cluster_list, endpoint_config);

    esp_err_t ret = esp_zb_device_register(esp_zb_ep_meter_list);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, , TAG, "Failed to register meter endpoint");

    esp_zb_core_action_handler_register(zb_action_handler);

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t current_summation_delivered_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = MY_METERING_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID,
        .flags = 0x0,
        .run_time = 0x00,
        .u.send_info.min_interval = 10,
        .u.send_info.max_interval = 60,
        .u.send_info.delta.u48 = {
            .low = 1,
            .high = 0
        },
        .u.send_info.reported_value.u48 = current_summation_delivered,
        .u.send_info.def_min_interval = 5,
        .u.send_info.def_max_interval = 30,
        .dst.endpoint = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&current_summation_delivered_reporting_info));

    esp_zb_zcl_reporting_info_t instantaneous_demand_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = MY_METERING_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID,
        .flags = 0x0,
        .run_time = 0x00,
        .u.send_info.min_interval = 10,
        .u.send_info.max_interval = 60,
        .u.send_info.delta.s24 = {
            .low = 1,
            .high = 0
        },
        .u.send_info.reported_value.s24 = instantaneous_demand,
        .u.send_info.def_min_interval = 5,
        .u.send_info.def_max_interval = 30,
        .dst.endpoint = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&instantaneous_demand_reporting_info));

    // esp_zb_zcl_reporting_info_t battery_reporting_info = {
    //     .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
    //     .ep = MY_METERING_ENDPOINT,
    //     .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
    //     .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //     .attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
    //     .flags = 0x0,
    //     .run_time = 0x00,
    //     .u.send_info.min_interval = 3600,
    //     .u.send_info.max_interval = (uint16_t)86400,
    //     .u.send_info.delta.u8 = 1,
    //     .u.send_info.reported_value.u8 = battery_voltage,
    //     .u.send_info.def_min_interval = 3600,
    //     .u.send_info.def_max_interval = (uint16_t)86400,
    //     .dst.endpoint = MY_METERING_ENDPOINT,
    //     .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    // };
    // ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&battery_reporting_info));

    // esp_zb_zcl_reporting_info_t percentage_reporting_info = {
    //     .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
    //     .ep = MY_METERING_ENDPOINT,
    //     .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
    //     .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //     .attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
    //     .flags = 0x0,
    //     .run_time = 0x00,
    //     .u.send_info.min_interval = 3600,
    //     .u.send_info.max_interval = (uint16_t)86400,
    //     .u.send_info.delta.u8 = 1,
    //     .u.send_info.reported_value.u8 = battery_percentage,
    //     .u.send_info.def_min_interval = 3600,
    //     .u.send_info.def_max_interval = (uint16_t)86400,
    //     .dst.endpoint = MY_METERING_ENDPOINT,
    //     .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    // };
    // ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&percentage_reporting_info));

    esp_zb_set_tx_power(-15);
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

// callback to start deep sleep
static void s_oneshot_timer_callback(void* arg)
{
    if (ignore_enter_sleep) {
        ESP_LOGI(TAG, "Enter deep sleep ignored");
    } else {
        /* Enter deep sleep */
        ESP_LOGI(TAG, "Enter deep sleep");
        gettimeofday(&s_sleep_enter_time, NULL);
        esp_deep_sleep_start();
    }
}

void leave_callback(esp_zb_zdp_status_t zdo_status, void* args)
{
    ESP_LOGI(TAG, "Leave status 0x%x", zdo_status);
}

// long press detected
static void s_longpress_timer_callback(void* arg)
{
    ESP_LOGI(TAG, "Button long press detected");
    uint16_t short_address = esp_zb_get_short_address();
    if (short_address != 0xfffe) {
        ESP_LOGI(TAG, "Leaving network");
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

// start timer to sleep
static void zb_deep_sleep_start(void)
{
    /* Start the one-shot timer */
    const int before_deep_sleep_time_sec = ACTIVE_TIME_AFTER_INTERRUPT;
    ESP_LOGI(TAG, "Start one-shot timer for %ds to enter the deep sleep", before_deep_sleep_time_sec);
    ESP_ERROR_CHECK(esp_timer_start_once(s_oneshot_timer, before_deep_sleep_time_sec * 1000000));
}

// restart timer to sleep to a new period
static void zb_deep_sleep_restart(void)
{
    const int before_deep_sleep_time_sec = ACTIVE_TIME_AFTER_INTERRUPT;
    ESP_LOGI(TAG, "Restart one-shot timer for %ds to enter the deep sleep", before_deep_sleep_time_sec);
    ESP_ERROR_CHECK(esp_timer_restart(s_oneshot_timer, before_deep_sleep_time_sec * 1000000));
}

static void zb_deep_sleep_start_or_restart(void)
{
    if (esp_timer_is_active(s_oneshot_timer))
        zb_deep_sleep_restart();
    else
        zb_deep_sleep_start();
}

static void zb_longpress_start(void) {
    int before_longpress_time_sec = LONG_PRESS_TIMEOUT * 1000;
    if (started_from_deep_sleep) {
        before_longpress_time_sec -= 2800; // measured time for the device to start
    }
    ESP_LOGI(TAG, "Start long press timer for %dms", before_longpress_time_sec);
    ESP_ERROR_CHECK(esp_timer_start_once(s_longpress_timer, before_longpress_time_sec * 1000));
}

static void zb_longpress_stop(void) {
    if (esp_timer_is_active(s_longpress_timer)) {
        ESP_LOGI(TAG, "Stop long press timer");
        ESP_ERROR_CHECK(esp_timer_stop(s_longpress_timer));
    }
}

// end zigbee values up if there are changes in the measured values
static void on_change_report_task(void *arg) {
    while (1) {
        if (check_gpio_time) {
            check_gpio_time = false;
            int level = gpio_get_level(PULSE_PIN);
            if (level == 0) { // avoid double counting
                zb_counter_increment();
                save_counter_nvr = true;
                last_pulse_time = gpio_time;
                force_report = true;
            }
        }
        if (force_report) {
            force_report = false;
            zb_radio_setup_values(true);
            zb_radio_send_values(true);
            if (save_counter_nvr) {
                zb_counter_save_nvs();
                restart_deep_sleep = true;
            }
        }
        if (restart_deep_sleep) {
            restart_deep_sleep = false;
            zb_deep_sleep_start_or_restart();
        }
        if (start_long_press_detector) {
            start_long_press_detector = false;
            zb_longpress_start();
        }
        if (stop_long_press_detector) {
            stop_long_press_detector = false;
            zb_longpress_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// After two consecutive values of current_summation_delivered this method
// computes the instantaneous_demand
// NOTE: there is one special task to set instantaneous demand to 0 when no
// values arrive
static void gm_compute_instantaneous_demand(int64_t current_time)
{
    if (last_pulse_time != 0) {
        int64_t time_diff_ms = (current_time - last_pulse_time) / 1000; // Tiempo en milisegundos
        // tested that max_unit32 can last for 49 days
        if (time_diff_ms > 0) {
            float time_diff_h = time_diff_ms / (1000.0 * 3600.0); // Convert time to hours/100
            int32_t _instantaneous_demand = (int32_t)((1 / time_diff_h) + 0.5); // compute flow in m³/h
            instantaneous_demand.low = _instantaneous_demand & 0x0000FFFF;
            instantaneous_demand.high = (_instantaneous_demand & 0x00FF0000) >> 16;
        }
    }
    last_pulse_time = current_time;
}

// update instantaneous demand to 0 when no pulses are received in a minute
static void gm_update_instantaneous_demand_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(interval * 1000));
        
        int64_t current_time = esp_timer_get_time();
        int64_t time_diff_ms = (current_time - last_pulse_time) / 1000;

        if (time_diff_ms > (interval * 1000)) {
            instantaneous_demand.low = 0;
            instantaneous_demand.high = 0;
            // ESP_LOGI(TAG, "No pulses detected in the last %d seconds, setting instantaneous demand to 0", interval);
            force_report = true;
        }
    }
}

// configure deep sleep for the gas meter
static void gm_deep_sleep_init(void)
{
    const esp_timer_create_args_t s_oneshot_timer_args = {
            .callback = &s_oneshot_timer_callback,
            .name = "one-shot"
    };

    ESP_ERROR_CHECK(esp_timer_create(&s_oneshot_timer_args, &s_oneshot_timer));

    const esp_timer_create_args_t s_longpress_timer_args = {
            .callback = &s_longpress_timer_callback,
            .name = "long-press"
    };

    ESP_ERROR_CHECK(esp_timer_create(&s_longpress_timer_args, &s_longpress_timer));

    // wake-up reason:
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - s_sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - s_sleep_enter_time.tv_usec) / 1000;
    esp_sleep_wakeup_cause_t wake_up_cause = esp_sleep_get_wakeup_cause();
    switch (wake_up_cause) {
    case ESP_SLEEP_WAKEUP_TIMER: {
        ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_EXT1: {
        ESP_LOGI(TAG, "Wake up from GPIO. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        int level = gpio_get_level(PULSE_PIN);
        if (level == 1) { // wakeup from PULSE_PIN
            gpio_time = esp_timer_get_time();
            check_gpio_time = true;
        }
        level = gpio_get_level(BUTTON_PIN);
        if (level == 1) { // wakeup from BUTTON_PIN
            start_long_press_detector = true;
            started_from_deep_sleep = true;
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGI(TAG, "Not a deep sleep reset");
        break;
    }

    /* Set the methods of how to wake up: */
    /* 1. RTC timer waking-up */
    const uint64_t wakeup_time_sec = 3600; // 1 hour
    ESP_LOGI(TAG, "Enabling timer wakeup, %llds", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

    /* PULSE_PIN pull up to wake up */
    const int gpio_wakeup_pin_1 = PULSE_PIN;
    const uint64_t gpio_wakeup_pin_mask_1 = 1ULL << gpio_wakeup_pin_1;
    const int gpio_wakeup_pin_2 = BUTTON_PIN;
    const uint64_t gpio_wakeup_pin_mask_2 = 1ULL << gpio_wakeup_pin_2;
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(gpio_wakeup_pin_mask_1 | gpio_wakeup_pin_mask_2, ESP_EXT1_WAKEUP_ANY_HIGH));
}

// tasks definitions to satisfy reporting requirements
static esp_err_t gm_tasks_init()
{
    return (
        xTaskCreate(on_change_report_task, "sensor_quick_update", 4096, NULL, 10, NULL) == pdTRUE
        && xTaskCreate(gm_update_instantaneous_demand_task, "instantaneous_update", 4096, NULL, 10, NULL) == pdTRUE
    ) ? ESP_OK : ESP_FAIL;
}

// Signal handler for zigbee radio
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    ESP_LOGI(TAG, "Shall handle signal 0x%x - %s", sig_type, esp_zb_zdo_signal_to_string(sig_type));
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", gm_tasks_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                can_restart_sleep = true;
                zb_deep_sleep_start_or_restart();
                ESP_LOGI(TAG, "Device rebooted");
            }
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
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
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
        ESP_LOGI(TAG, "Can sleep");
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

// GPIO Interruption handler for the magnetic sensor
void IRAM_ATTR gpio_pulse_isr_handler(void* arg) {
    // Increment current_summation_delivery
    // read pin to determine failing or raising edge
    int level = gpio_get_level(PULSE_PIN);
    if (level == 0) {
        // failing
        int64_t current_time = esp_timer_get_time();
        int64_t time_diff_ms = (current_time - last_pulse_time) / 1000;
        if (last_pulse_time > 0 && time_diff_ms <= DEBOUNCE_TIMEOUT)
            return; // DEBOUNCE
        
        zb_counter_increment();
        gm_compute_instantaneous_demand(current_time);

        force_report = true;
        save_counter_nvr = true;
        ignore_enter_sleep = false;
        check_gpio_time = false; // avoid double count

        last_pulse_time = current_time; // Actualizar el tiempo del último pulso
    } else {
        // rising, the device was NOT in deep sleep mode. At this moment, if there
        // is no activity in 2 minutes, the device will try to enter deep sleep but
        // it won't success because the pin level is already high so it will wake up
        // inmediatelly. The solution at this moment is to stop the timer interrupt
        // until the failing edge is detected
        ignore_enter_sleep = true;
    }
}

// GPIO Interruption handler for the main button
void IRAM_ATTR gpio_wakeup_isr_handler(void *arg) {
    int level = gpio_get_level(BUTTON_PIN);
    int64_t current_time = esp_timer_get_time();
    int64_t time_diff_ms = (current_time - last_wakeup_time) / 1000;
    if (level == 1) {
        if (last_pulse_time > 0 && time_diff_ms <= DEBOUNCE_TIMEOUT)
            return; // DEBOUNCE

        if (can_restart_sleep) {
            restart_deep_sleep = true;
            force_report = true;
            start_long_press_detector = true;
            started_from_deep_sleep = false;
        }
    }
    if (level == 0) {
        // detect long and short time press
        stop_long_press_detector = true;
    }

    last_wakeup_time = current_time;
}

// Configure interrupt to catch pulses from magnetic sensor
// NOTE: When the device is in deep sleep internal pull-up and pull-down 
// resistors can't be used so they must be disabled.
// In order to save battery I decided to use pull-down because this is
// how the sensor should stay most of the time
void gm_gpio_interrup_init() {
    uint64_t pulse_pin  = 1ULL << PULSE_PIN;
    uint64_t wakeup_pin = 1ULL << BUTTON_PIN;
                                        //      __
    gpio_config_t io_conf = {           // ____|  |_____
        .intr_type = GPIO_INTR_ANYEDGE, //     ^--^- Interrupt both edges
        .mode = GPIO_MODE_INPUT,        // Input pin
        .pin_bit_mask = pulse_pin | wakeup_pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistor
        .pull_up_en = GPIO_PULLUP_DISABLE    // Disable pull-up resistor
    };
    gpio_config(&io_conf);

    // Configure and register interrupt service
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    gpio_isr_handler_add(PULSE_PIN, gpio_pulse_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_PIN, gpio_wakeup_isr_handler, NULL);
}

// Entry point
void app_main(void) {
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "Starting Zigbee GasMeter...");

    gm_gpio_interrup_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(zb_counter_load_nvs());

    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    gm_deep_sleep_init();

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}