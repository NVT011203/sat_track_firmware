#include "sx1278.h"

// SPI write function
void sx1278_write(uint8_t addr, uint8_t value) {
  uint8_t data[2] = {addr | 0x80, value};
  spi_transaction_t t = {
      .length = 16,
      .tx_buffer = data,
  };
  spi_device_transmit(spi, &t);
}

// SPI read function
uint8_t sx1278_read(uint8_t addr) {
  uint8_t data_out[2] = {addr & 0x7F, 0x00};
  uint8_t data_in[2];
  spi_transaction_t t = {
      .length = 16,
      .tx_buffer = data_out,
      .rx_buffer = data_in,
  };
  spi_device_transmit(spi, &t);
  return data_in[1];
}

void sx1278_configure_fsk() {
  sx1278_write(REG_OP_MODE, 0x00);

  sx1278_write(REG_FR_MSB, 0x6C);
  sx1278_write(REG_FR_MID, 0x80);
  sx1278_write(REG_FR_LSB, 0x00);

  sx1278_write(REG_BITRATE_MSB, 0x1A);
  sx1278_write(REG_BITRATE_LSB, 0x0B);
  sx1278_write(REG_FDEV_MSB, 0x00);
  sx1278_write(REG_FDEV_LSB, 0x52);
  sx1278_write(REG_RX_BW, 0x55);
  sx1278_write(REG_PAYLOAD_LENGTH, 0x40);
  sx1278_write(REG_OP_MODE, 0x01);
}

void sx1278_start_rx() {
  sx1278_write(REG_IRQ_FLAGS, 0xFF);
  sx1278_write(REG_OP_MODE, 0x10);
}

void sx1278_read_data() {
  uint8_t irq_flags = sx1278_read(REG_IRQ_FLAGS);
  if (irq_flags & 0x40) {
    ESP_LOGI(SX1278_TAG, "Packet received!");

    sx1278_write(REG_IRQ_FLAGS, 0x40);

    uint8_t length = sx1278_read(REG_PAYLOAD_LENGTH);

    uint8_t data[length];
    for (int i = 0; i < length; i++) {
      data[i] = sx1278_read(REG_FIFO);
    }

    ESP_LOGI(SX1278_TAG, "Data: ");
    for (int i = 0; i < length; i++) {
      printf("%02X ", data[i]);
    }
    printf("\n");
  } else {
    ESP_LOGI(SX1278_TAG, "No packet received.");
  }
}

void spi_init() {
  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 256,
  };

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 10 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS,
      .queue_size = 7,
  };

  spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

  sx1278_configure_fsk();
  sx1278_start_rx();
}

void test_sx1278() {
  while (1) {
    sx1278_read_data();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}