idf_component_register(SRCS "lcd_bl_pwm_bsp.c"
                       INCLUDE_DIRS "." 
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}/main"
                       PRIV_REQUIRES esp_driver_ledc esp_driver_gpio driver)
