idf_component_register(
    SRCS "led_ring.cpp"
    INCLUDE_DIRS "include"
    REQUIRES led_strip nvs_config espp
)
