#ifndef MPU9250
#define MPU9250

// Include libraries
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "euler_angles.h"
#include "i2c_init.h"
#include <stdio.h>

// I2C address of MPU9250
#define MPU9250_ADDR 0x68
// MPU9250 register addresses
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// MPU9250
#define MPU9250_TAG "MPU9250" // MPU TAG
static esp_err_t mpu9250_read_bytes(uint8_t reg_addr, uint8_t *data,
                                    size_t size, uint8_t I2C_MASTER_PORT);
static void mpu9250_read_sensor_data(Sensor_data *accel, Sensor_data *gyro);

#endif