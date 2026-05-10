idf_component_register(
    SRCS "pressure_sensor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver led_ring nvs_config espp
)
