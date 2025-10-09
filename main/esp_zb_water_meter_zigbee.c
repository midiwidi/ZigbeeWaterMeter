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
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "hal/ieee802154_ll.h"

#include "esp_zb_water_version.h"
#include "esp_zb_water_meter.h"
#include "esp_zb_water_meter_zigbee.h"
#include "esp_zb_water_meter_adc_zigbee.h"
#include "esp_zb_water_ota.h"

/* Zigbee configuration */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */
#define MY_METERING_ENDPOINT            1

#define ESP_MANUFACTURER_NAME           "\x08""MIDIWIDI"
#define ESP_MODEL_IDENTIFIER            "\x0A""WaterMeter" /* Customized model identifier */
#define ESP_DATE_CODE                   "\x08""20251006"
#define ESP_PRODUCT_URL                 "\x2C""https://github.com/midiwidi/ZigbeeWaterMeter"

esp_zb_uint48_t current_summation_delivered = {
	.low = 0,
	.high = 0
};
uint8_t device_status = 0x0;
uint64_t device_extended_status = 0x0;
esp_zb_int24_t instantaneous_demand = {
    .low = 0,
    .high = 0
};

struct timeval last_report_sent_time = {0};
uint64_t last_summation_sent = 0;

// value for the ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID attribute
uint16_t identify_time = 0;

uint8_t battery_alarm_mask = 0x03;

uint8_t battery_voltage_rated = 74;

uint8_t battery_quantity = 2;

// value for the manufacturer_code, At this time this is whatever value 
// I've never seen before. I don't know if there is a value for DIY devices
uint16_t manufacturer_code = HW_MANUFACTURER_CODE;

// Formatting current_summation_delivered with 7 digits and 0 decimal places
uint8_t summation_formatting = ESP_ZB_ZCL_METERING_FORMATTING_SET(true, 7, 0) ;

// Formatting instantaneous demand with 1 decimal places
uint8_t demand_formatting = ESP_ZB_ZCL_METERING_FORMATTING_SET(true, 4, 1) ;

// Water metering type
uint8_t metering_device_type = ESP_ZB_ZCL_METERING_WATER_METERING; 

// m³
uint8_t unit_of_measure = ESP_ZB_ZCL_METERING_UNIT_L_LH_BINARY; 

// 1 liter for every 1 pulse
esp_zb_uint24_t multiplier = {
    .low = 1,
    .high = 0
};

// 1 pulse is 1 liter
esp_zb_uint24_t divisor = {
    .low = 1,
    .high = 0
};

uint8_t alarm_mask = 0x03;
uint8_t generic_device_type = 0xFF;

// response to the ESP_ZB_CORE_REPORT_ATTR_CB_ID callback
esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
        message->status);
    ESP_LOGI(TAG, "Report received from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x) attribute(0x%x) type(0x%x)",
        message->src_address.u.short_addr, 
        message->src_endpoint,
        message->dst_endpoint, 
        message->cluster,
        message->attribute.id,
        message->attribute.data.type
    );
    return ESP_OK;
}

// response to the ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID callback: TODO check if needed
esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x) transaction(0x%x)",
             message->info.src_address.u.short_addr, 
             message->info.src_endpoint,
             message->info.dst_endpoint, 
             message->info.cluster,
             message->info.header.tsn
    );

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response variable: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
            variable->status, 
            message->info.cluster,
            variable->attribute.id, 
            variable->attribute.data.type,
            variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0
        );
        variable = variable->next;
    }

    return ESP_OK;
}

void zb_log_esp_zb_zcl_cmd_info_t(const char* prefix, const esp_zb_zcl_cmd_info_t *info)
{
    ESP_LOGI(TAG, "%s:\n  From address(0x%x) src endpoint(%d)\n  To address (0x%x) endpoint(%d)\n    cluster(0x%x) profile(0x%x) transaction(%d)\n    cmd id(0x%x) direction(0x%x) is_common(0x%x)",
        prefix,
        info->src_address.u.short_addr, 
        info->src_endpoint,
        info->dst_address, 
        info->dst_endpoint, 
        info->cluster,
        info->profile,
        info->header.tsn,
        info->command.id,
        info->command.direction,
        info->command.is_common
    );
}

// response to the ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID callback: TODO check if needed
esp_err_t zb_write_attr_resp_handler(const esp_zb_zcl_cmd_write_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Write attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x) transaction(%d)",
        message->info.src_address.u.short_addr, 
        message->info.src_endpoint,
        message->info.dst_endpoint, 
        message->info.cluster,
        message->info.header.tsn
    );

    esp_zb_zcl_write_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Write attribute response: status(%d), attribute(0x%x)",
                    variable->status,
                    variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

// response to report config command
esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    zb_log_esp_zb_zcl_cmd_info_t("Configure report response", &message->info);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
            variable->status, 
            message->info.cluster, 
            variable->direction, 
            variable->attribute_id
        );
        variable = variable->next;
    }

    return ESP_OK;
}

esp_err_t zb_cmd_default_resp_handler(const esp_zb_zcl_cmd_default_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    zb_log_esp_zb_zcl_cmd_info_t("Command default response", &message->info);

    ESP_LOGI(TAG, "Default Response: Status: 0x%04x, To Command: 0x%x", message->status_code, message->resp_to_cmd);

    return ESP_OK;
}

// transfer values from variables to zigbee radio
esp_zb_zcl_status_t zb_radio_setup_report_values(EventBits_t uxBits)
{
    esp_zb_zcl_status_t status = ESP_ZB_ZCL_STATUS_SUCCESS;
    if ((uxBits & CURRENT_SUMMATION_DELIVERED_REPORT) != 0) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID, &current_summation_delivered, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of current summation delivered: %d", status);
            return status;
        }
    }
    if ((uxBits & INSTANTANEOUS_DEMAND_REPORT) != 0) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID, &instantaneous_demand, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of instantaneous demand: %d", status);
            return status;
        }
    }
    if ((uxBits & BATTER_REPORT) != 0) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_percentage, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of battery percentage: %d", status);
        }
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of battery voltage: %d", status);
        }
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID, &battery_alarm_state, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of battery alarm state: %d", status);
        }
    }
    if ((uxBits & STATUS_REPORT) != 0) {
        status = esp_zb_zcl_set_attribute_val(MY_METERING_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_METERING_STATUS_ID, &device_status, false);
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Updating value of status: %d", status);
        }
    }
    if ((uxBits & EXTENDED_STATUS_REPORT) != 0) {
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
void zb_radio_send_values(EventBits_t uxBits)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = MY_METERING_ENDPOINT;

    if ((uxBits & CURRENT_SUMMATION_DELIVERED_REPORT) != 0) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "CurrentSummationDelivered reported");
    }

    if ((uxBits & INSTANTANEOUS_DEMAND_REPORT) != 0) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "InstantaneousDemand reported");
    }

    if ((uxBits & BATTER_REPORT) != 0) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "BatteryPercentageRemaining reported");
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "BatteryAlarmState reported");
    }

    if ((uxBits & STATUS_REPORT) != 0) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_STATUS_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "Status reported");
    }

    if ((uxBits & EXTENDED_STATUS_REPORT) != 0) {
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_METERING_EXTENDED_STATUS_ID;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_METERING;
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));
        ESP_LOGI(TAG, "ExtendedStatus reported");
    }
}

// Attribute handler
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) 
{
    ESP_LOGI(TAG, "In zb_action_handler callback_id=0x%04x", callback_id);
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
            setAtrMsg->attribute.id == ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID &&
            setAtrMsg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_METERING &&
            setAtrMsg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U48 &&
            setAtrMsg->attribute.data.size == sizeof(esp_zb_uint48_t)
        ) {
            gm_counter_set(setAtrMsg->attribute.data.value);
        }
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID: // 4101
        ret = zb_cmd_default_resp_handler((esp_zb_zcl_cmd_default_resp_message_t*)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID: // 0x0004
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
        ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%04x) callback", callback_id);
        break;
    }
    return ret;
}

void zb_command_handler(esp_zb_zcl_command_send_status_message_t message)
{
    ESP_LOGI(TAG, "In zb_command_handler");
    ESP_LOGI(TAG, "  status(0x%x) transaction(%d)", message.status, message.tsn);
}

// bool zb_raw_command_handler(uint8_t bufid)
// {
//     ESP_LOGI(TAG, "In zb_raw_command_handler");
//     ESP_LOGI(TAG, "  bufid(0x%x)", bufid);

//     return false;
// }

// bool zb_device_cb_id(uint8_t bufid)
// {
//     ESP_LOGI(TAG, "In zb_device_cb_id");
//     ESP_LOGI(TAG, "  bufid(0x%x)", bufid);

//     return false;
// }

// initialize zigbee device
void esp_zb_task(void *pvParameters) 
{
    ESP_LOGI(TAG, "In esp_zb_task");

    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

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
        .zcl_version = zigbee_zcl_version,
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
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, 
        ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_version
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
        .main_voltage = 74,
        .main_freq = 0,
        .main_alarm_mask = 0x03,
        .main_voltage_min = 70,
        .main_voltage_max = 84,
        .main_voltage_dwell = MUST_SYNC_MINIMUM_TIME
    };

    esp_zb_attribute_list_t *esp_zb_power_cluster = esp_zb_power_config_cluster_create(&power_cfg);
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                                        &battery_voltage));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                        &battery_percentage));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID,
                                        &battery_alarm_state));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID,
                                        &battery_alarm_mask));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID,
                                        &battery_voltage_min));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_THRESHOLD1_ID,
                                        &battery_voltage_th1));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID,
                                        &battery_voltage_rated));

    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(esp_zb_power_cluster,
                                        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID,
                                        &battery_quantity));

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

    /* ota cluster */
    // const esp_app_desc_t *app_desc = esp_app_get_description();

    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        // .ota_upgrade_file_version = app_desc->secure_version,
        .ota_upgrade_file_version = OTA_FILE_VERSION,
        .ota_upgrade_manufacturer = manufacturer_code,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_STACK_VERSION_ID, &zigbee_stack_version));
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(esp_zb_ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &variable_config));

    /* create cluster lists for this endpoint */
    esp_zb_cluster_list_t *esp_zb_meter_cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(esp_zb_meter_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_meter_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(esp_zb_meter_cluster_list, esp_zb_identify_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_metering_cluster(esp_zb_meter_cluster_list, esp_zb_metering_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(esp_zb_meter_cluster_list, esp_zb_power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(esp_zb_meter_cluster_list, esp_zb_ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

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
    // ESP_LOGI(TAG, "Registering additional callbacks");
    // esp_zb_device_cb_id_handler_register(zb_device_cb_id);
    // esp_zb_raw_command_handler_register(zb_raw_command_handler);
    // esp_zb_zcl_command_send_status_handler_register(zb_command_handler);
    // ESP_LOGI(TAG, "Additional callbacks registered");

    esp_zb_zcl_attr_location_info_t current_summation_delivered_location_info = {
        .attr_id = ESP_ZB_ZCL_ATTR_METERING_CURRENT_SUMMATION_DELIVERED_ID,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_METERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .endpoint_id = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };
    ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(current_summation_delivered_location_info));
    esp_zb_zcl_reporting_info_t *current_summation_reporting_info = esp_zb_zcl_find_reporting_info(current_summation_delivered_location_info);
    current_summation_reporting_info->u.send_info.max_interval = (uint16_t)MUST_SYNC_MINIMUM_TIME;
    current_summation_reporting_info->u.send_info.min_interval = TIME_TO_SLEEP_ZIGBEE_ON / 2000;
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

    esp_zb_zcl_attr_location_info_t  battery_alarm_state_location_info = {
        .attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .endpoint_id = MY_METERING_ENDPOINT,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };
    ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(battery_alarm_state_location_info));

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
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(esp_zb_set_secondary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK));

    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Starting Zigbee Main Loop");
    esp_zb_stack_main_loop();
}

// top level comissioning callback
void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

// end zigbee values up if there are changes in the measured values
void gm_main_loop_zigbee_task(void *arg) 
{
    ESP_LOGI(TAG, "Zigbee Loop Task...");
    while (true) {
        // Not all bits will be set, but it is more efficient to wait for the maximum
        // number of bits to be set before continuing, this is the reason we wait for
        // all bits to be set.
        EventBits_t uxBits = xEventGroupWaitBits(
            report_event_group_handle, 
            CURRENT_SUMMATION_DELIVERED_REPORT | INSTANTANEOUS_DEMAND_REPORT | STATUS_REPORT | EXTENDED_STATUS_REPORT | BATTER_REPORT,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(250)
        );
        if (uxBits != 0) {
            // Note we must manually clear the bits to avoid infinite loop
            xEventGroupClearBits(report_event_group_handle, uxBits);
            ESP_LOGI(TAG, "Reporting to client Sum=%s, Instant=%s, Bat=%s, Status=%s, Exten=%s", 
                ((uxBits & CURRENT_SUMMATION_DELIVERED_REPORT) != 0) ? "Yes": "No",
                ((uxBits & INSTANTANEOUS_DEMAND_REPORT) != 0) ? "Yes": "No",
                ((uxBits & BATTER_REPORT) != 0) ? "Yes": "No",
                ((uxBits & STATUS_REPORT) != 0) ? "Yes": "No",
                ((uxBits & EXTENDED_STATUS_REPORT) != 0) ? "Yes": "No"
            );
            if (esp_zb_lock_acquire(portMAX_DELAY)) 
            {
                esp_zb_zcl_status_t status = zb_radio_setup_report_values(uxBits);
                if (status == ESP_ZB_ZCL_STATUS_SUCCESS) {
                    zb_radio_send_values(uxBits);
                }
                esp_zb_lock_release();
                gettimeofday(&last_report_sent_time, NULL);
                last_summation_sent = current_summation_delivered.high;
                last_summation_sent <<= 32;
                last_summation_sent |= current_summation_delivered.low;
            }
            if (deep_sleep_task_handle != NULL) {
                TickType_t deep_sleep_time = dm_deep_sleep_time_ms();
                if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                    ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
            }
        }
    }
}

// tasks definitions to satisfy reporting requirements
esp_err_t gm_tasks_init()
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
    ESP_LOGV(TAG, "Shall handle signal 0x%x - %s", sig_type, esp_zb_zdo_signal_to_string(sig_type));
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGD(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
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
        ESP_LOGV(TAG, "Can sleep");
        break;
    default:
        ESP_LOGD(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}
