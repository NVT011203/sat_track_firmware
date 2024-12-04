
#include "mpu9250.h"

// MPU9250
static esp_err_t mpu9250_read_bytes(uint8_t reg_addr, uint8_t *data, size_t size, uint8_t I2C_MASTER_PORT)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void mpu9250_read_sensor_data(Sensor_data *accel, Sensor_data *gyro)
{
    uint8_t data[14];
    esp_err_t ret = mpu9250_read_bytes(ACCEL_XOUT_H, data, 14, I2C_MASTER_PORT);
    if (ret == ESP_OK)
    {
        // Đọc dữ liệu gia tốc
        accel->X = (data[0] << 8) | data[1];
        accel->Y = (data[2] << 8) | data[3];
        accel->Z = (data[4] << 8) | data[5];

        // Đọc dữ liệu con quay hồi chuyển
        gyro->X = (data[8] << 8) | data[9];
        gyro->Y = (data[10] << 8) | data[11];
        gyro->Z = (data[12] << 8) | data[13];
    }
    else
    {
        ESP_LOGE(MPU9250_TAG, "Error reading sensor data: %s", esp_err_to_name(ret));
    }
}
