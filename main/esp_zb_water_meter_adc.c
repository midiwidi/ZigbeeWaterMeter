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
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "zcl/esp_zigbee_zcl_metering.h"

#include "esp_zb_water_meter.h"
#include "esp_zb_water_meter_zigbee.h"
#include "esp_zb_water_meter_adc.h"
#include "esp_zb_water_meter_adc_zigbee.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BAT_DIVIDER_R1      120000  // 120k resistor to battery positive
#define BAT_DIVIDER_R2      38000   // 38k resistor to ground
#define BAT_DIVIDER_RATIO   ((float)(BAT_DIVIDER_R1 + BAT_DIVIDER_R2) / BAT_DIVIDER_R2)

uint8_t battery_voltage = 0;
uint8_t battery_percentage = 0;
uint32_t battery_alarm_state = 0;
uint8_t battery_voltage_min = 33;
uint8_t battery_voltage_th1 = 34;

struct timeval last_battery_measurement_time;

adc_channel_t channel = ADC_CHANNEL_0;
adc_continuous_handle_t handle = NULL;

// Lookup Table for battery voltage -> percentage conversion
static const struct {
    uint8_t percentage;
    float voltage;
} battery_lut[] = {
    {100, 4.20}, {95, 4.15}, {90, 4.11}, {85, 4.08}, {80, 4.02},
    {75, 3.98}, {70, 3.95}, {65, 3.91}, {60, 3.87}, {55, 3.85},
    {50, 3.84}, {45, 3.82}, {40, 3.80}, {35, 3.79}, {30, 3.77},
    {25, 3.75}, {20, 3.73}, {15, 3.71}, {10, 3.69}, {5, 3.61},
    {0, 3.27}
};

static uint8_t interpolate_battery_percentage(float voltage) {
    // Handle out of bounds cases
    if (voltage >= battery_lut[0].voltage) return 100;
    if (voltage <= battery_lut[20].voltage) return 0;
    
    // Find the voltage range
    for (int i = 0; i < 20; i++) {
        if (voltage <= battery_lut[i].voltage && voltage > battery_lut[i + 1].voltage) {
            // Linear interpolation
            float v_delta = battery_lut[i].voltage - battery_lut[i + 1].voltage;
            float p_delta = battery_lut[i].percentage - battery_lut[i + 1].percentage;
            float v_offset = battery_lut[i].voltage - voltage;
            return battery_lut[i].percentage - (uint8_t)((v_offset * p_delta) / v_delta + 0.5f);
        }
    }
    return 0;
}

esp_err_t bat_adc_calibration_init(const adc_unit_t unit, const adc_channel_t channel, const adc_atten_t atten, adc_cali_handle_t *out_handle)
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

esp_err_t bat_adc_calibration_deinit(const adc_cali_handle_t handle)
{
    ESP_LOGD(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_RETURN_ON_ERROR(adc_cali_delete_scheme_curve_fitting(handle), TAG, "Failed to delete ADC calibration curve");
    return ESP_OK;
}

bool IRAM_ATTR bat_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

void bat_continuous_adc_init(const adc_channel_t channel, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = BAT_BUFFER_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_pattern[0].atten = ADC_ATTEN_DB_0;
    adc_pattern[0].channel = channel & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

// adc task. Obtains battery voltage from adc conversion and formula.
// The task is deleted when the process finish
void adc_task(void *arg)
{
    ESP_LOGD(TAG, "ADC Task Init...");
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;

    //gpio_set_level(BAT_MON_ENABLE, 1);
    bat_continuous_adc_init(channel, &handle);
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = bat_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    ESP_ERROR_CHECK(bat_adc_calibration_init(ADC_UNIT_1, channel, ADC_ATTEN_DB_0, &adc1_cali_chan0_handle));

    uint8_t result[BAT_BUFFER_READ_LEN] = {0};
    memset(result, 0xcc, BAT_BUFFER_READ_LEN);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //gpio_set_level(BAT_MON_ENABLE, 0);

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
            
            // voltage is in mV, convert to battery voltage using divider ratio
            float bat_voltage = (float)voltage * BAT_DIVIDER_RATIO / 1000.0f;
            
            // Store as decivolts (multiply by 10 and round)
            battery_voltage = (uint8_t)(bat_voltage * 10.0f + 0.5f);
            
            // Calculate percentage using lookup table with actual voltage
            battery_percentage = interpolate_battery_percentage(bat_voltage);
            
            // Scale battery percentage for Zigbee (0-200 represents 0-100%)
            battery_percentage *= 2;
            
            xEventGroupSetBits(report_event_group_handle, BATTER_REPORT);

            if (battery_voltage < battery_voltage_th1) {
                battery_alarm_state |= (1 << 1); // BatteryVoltageThreshold1 or BatteryPercentageThreshold1 reached for Battery Source 1
            } else {
                battery_alarm_state &= ~(1 << 1);
            }
            if (battery_voltage < battery_voltage_min) {
                battery_alarm_state |= (1 << 0); // BatteryVoltageMinThreshold or BatteryPercentageMinThreshold reached for Battery Source 1
            } else {
                battery_alarm_state &= ~(1 << 0);
            }
            if (battery_alarm_state != 0 && (device_status & ESP_ZB_ZCL_METERING_WATER_LOW_BATTERY) == 0) {
                device_status |= ESP_ZB_ZCL_METERING_WATER_LOW_BATTERY;
                xEventGroupSetBits(report_event_group_handle, STATUS_REPORT);
            } else if (battery_alarm_state == 0 && (device_status & ESP_ZB_ZCL_METERING_WATER_LOW_BATTERY) != 0) {
                device_status &= ~ESP_ZB_ZCL_METERING_WATER_LOW_BATTERY;
                xEventGroupSetBits(report_event_group_handle, STATUS_REPORT);
            }

            ESP_LOGI(TAG, "Raw: %"PRIu32" Calibrated: %"PRId16"mV Bat Voltage: %1.2fv ZB Voltage: %d ZB Percentage: %d Alarm 0x%lx", 
                average, voltage, bat_voltage, battery_voltage, battery_percentage, battery_alarm_state);
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
    adc_task_handle = NULL;
    vTaskDelete(NULL);
}
