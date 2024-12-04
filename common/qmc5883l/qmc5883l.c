#include "qmc5883l.h"
#include "i2c_init.h"

// HMC5883L
esp_err_t qmc5883l_write_byte(uint8_t reg_addr, uint8_t data, uint8_t I2C_MASTER_PORT)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t qmc5883l_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len, uint8_t I2C_MASTER_PORT)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to initialize QMC5883L
void qmc5883l_init()
{
    ESP_ERROR_CHECK(qmc5883l_write_byte(QMC5883L_REG_SET_RESET, 0x01, I2C_MASTER_PORT)); // Perform soft reset
    ESP_ERROR_CHECK(qmc5883l_write_byte(QMC5883L_REG_CONTROL, 0x1D, I2C_MASTER_PORT));   // Continuous mode, 200Hz, RNG=8G, OSR=512
}

void qmc5883l_read_magnetometer(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    if (qmc5883l_read_bytes(QMC5883L_REG_X_LSB, data, 6, I2C_MASTER_PORT) == ESP_OK)
    {
        *x = (int16_t)((data[1] << 8) | data[0]);
        *z = (int16_t)((data[3] << 8) | data[2]);
        *y = (int16_t)((data[5] << 8) | data[4]);
    }
    else
    {
        ESP_LOGE(QMC_TAG, "Failed to read data from QMC5883L");
    }
}
