#ifndef MPU9250
#define MPU9250

// Include libraries
#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "euler_angles.h"
#include "i2c_init.h"

// I2C address of MPU9250
#define MPU9250_ADDR 0x68
// MPU9250 register addresses
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
// #define PWR_MGMT_1 0x6B
// #define WHO_AM_I 0x75

// MPU9250
#define MPU9250_TAG "MPU9250"                                                                               // MPU TAG
static esp_err_t mpu9250_read_bytes(uint8_t reg_addr, uint8_t *data, size_t size, uint8_t I2C_MASTER_PORT); // Đọc dữ liệu từ thanh ghi
static void mpu9250_read_sensor_data(Sensor_data *accel, Sensor_data *gyro);                                // Đọc dữ liệu từ MPU9250

#endif