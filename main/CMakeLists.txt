idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        dust_sensor
        rf_remote
        fan_control
        display
        nvs_config
        esp_wifi
        esp_https_ota
        nvs_flash
        esp_task_wdt
        espp
)
