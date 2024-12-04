idf_component_register(SRCS "mpu9250.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c driver
                       PRIV_REQUIRES i2c calc)