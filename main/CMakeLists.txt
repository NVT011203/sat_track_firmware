idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES calc gps mpu9250 qmc5883l i2c stepper_motor data_transform sx1278
                       PRIV_REQUIRES esp_wifi spi_flash esp_event nvs_flash app_update esp-tls mqtt)