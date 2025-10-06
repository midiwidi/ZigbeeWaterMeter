idf_component_register(
        SRCS "esp_zb_water_ota.c" "esp_zb_water_meter_zigbee.c" "esp_zb_water_meter.c" "esp_zb_water_meter_adc.c" "esp_zb_water_meter_zigbee.c" "esp_zb_water_ota.c"
        INCLUDE_DIRS "."
        REQUIRES esp-zigbee-lib nvs_flash esp_timer esp_driver_gpio esp_adc app_update
)
