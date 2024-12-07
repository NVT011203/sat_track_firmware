#ifndef QMC5883L
#define QMC5883L

// Include libraries
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>

#define QMC5883L_ADDRESS 0x0D // I2C address of QMC5883L
// QMC5883L register addresses
#define QMC5883L_REG_X_LSB 0x00
#define QMC5883L_REG_X_MSB 0x01
#define QMC5883L_REG_Y_LSB 0x02
#define QMC5883L_REG_Y_MSB 0x03
#define QMC5883L_REG_Z_LSB 0x04
#define QMC5883L_REG_Z_MSB 0x05
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_CONTROL 0x09
#define QMC5883L_REG_SET_RESET 0x0B

// Define new error check function
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                                                     \
  do {                                                                         \
    esp_err_t rc = (x);                                                        \
    if (rc != ESP_OK) {                                                        \
      ESP_LOGE("err", "esp_err_t = %d", rc);                                   \
      assert(0 && #x);                                                         \
    }                                                                          \
  } while (0);

// QMC5883L function
#define QMC_TAG "QMC5883L"
esp_err_t qmc5883l_write_byte(uint8_t reg_addr, uint8_t data,
                              uint8_t I2C_MASTER_PORT); // Write function
esp_err_t qmc5883l_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len,
                              uint8_t I2C_MASTER_PORT); // Read function
void qmc5883l_init();                                   // Init QMC5883L
void qmc5883l_read_magnetometer(int16_t *x, int16_t *y,
                                int16_t *z); // Read magnetic from QMC5883L

// End
#endif