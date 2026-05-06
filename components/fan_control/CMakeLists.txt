idf_component_register(
    SRCS "fan_control.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver led_ring nvs_config rf_remote espp
)
