idf_component_register(SRCS "lcd_touch_bsp.c"
                       INCLUDE_DIRS "." 
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}/main"
                        PRIV_REQUIRES esp_driver_i2c esp_driver_gpio i2c_bsp lcd_bl_pwm_bsp driver)
