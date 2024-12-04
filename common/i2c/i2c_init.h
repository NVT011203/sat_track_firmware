#ifndef I2C_INIT
#define I2C_INIT

// Include libraries
#include "driver/i2c_master.h"
#include "driver/i2c.h"

// Define
#define I2C_TAG "I2C"

#define I2C_MASTER_SCL_IO 22      // SCL pin
#define I2C_MASTER_SDA_IO 21      // SDA pin
#define I2C_MASTER_FREQ_HZ 100000 // Frequency 100Hz
#define I2C_MASTER_PORT I2C_NUM_0 // I2C port

#define static esp_err_t i2c_master_init(void); // I2C initialize

#endif