idf_component_register(SRCS "i2c_bsp.c"
                       INCLUDE_DIRS "." 
                       PRIV_INCLUDE_DIRS "${PROJECT_DIR}/main"   # <-- Changed to ${PROJECT_DIR}
                       PRIV_REQUIRES esp_driver_i2c)
