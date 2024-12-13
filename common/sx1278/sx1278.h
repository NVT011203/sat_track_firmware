#ifndef SX1278
#define SX1278

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


#define SX1278_TAG "SX1278"
spi_device_handle_t spi;

// SPI Pin definitions
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
#define PIN_NUM_RST 14
#define PIN_NUM_IRQ 4

// SX1278 Registers
#define REG_OP_MODE 0x01
#define REG_FR_MSB 0x06
#define REG_FR_MID 0x07
#define REG_FR_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_BITRATE_MSB 0x02
#define REG_BITRATE_LSB 0x03
#define REG_FDEV_MSB 0x04
#define REG_FDEV_LSB 0x05
#define REG_RX_BW 0x12
#define REG_IRQ_FLAGS 0x12
#define REG_PAYLOAD_LENGTH 0x22
#define REG_FIFO 0x00

// SPI write function
void sx1278_write(uint8_t addr, uint8_t value);
// SPI read function
uint8_t sx1278_read(uint8_t addr);
void sx1278_configure_fsk();
void sx1278_start_rx();
void sx1278_read_data();
void spi_init();
void test_sx1278();

#endif
