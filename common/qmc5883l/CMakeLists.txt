idf_component_register(SRCS "qmc5883l.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c driver
                       PRIV_REQUIRES i2c)