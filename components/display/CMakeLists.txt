idf_component_register(
    SRCS "display.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_lcd nvs_config rf_remote espp
)
