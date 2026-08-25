idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_ledc esp_driver_gpio driver i2c_bsp lcd_touch_bsp lcd_bl_pwm_bsp user_encoder_bsp esp_driver_spi esp_timer)

set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)
