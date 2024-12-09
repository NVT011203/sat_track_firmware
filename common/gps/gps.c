#include "gps.h"

float convert_to_decimal(char *raw, char direction) {
  float degrees = 0.0;
  float minutes = 0.0;

  if (strlen(raw) > 4) {
    char degrees_str[3] = {0};
    strncpy(degrees_str, raw, (direction == 'N' || direction == 'S') ? 2 : 3);
    degrees = atof(degrees_str);

    minutes = atof(raw + ((direction == 'N' || direction == 'S') ? 2 : 3));
  }

  float result = degrees + (minutes / 60.0);
  if (direction == 'S' || direction == 'W') {
    result = -result;
  }
  return result;
}

gps_data_t parse_gpgga(const char *sentence) {
  gps_data_t data = {0.0, 0.0, 0.0};
  char latitude_raw[16] = {0}, longitude_raw[16] = {0}, lat_dir = 0,
       lon_dir = 0, altitude_raw[16] = {0};

  sscanf(sentence, "$GPGGA,%*f,%[^,],%c,%[^,],%c,%*d,%*d,%*f,%[^,],",
         latitude_raw, &lat_dir, longitude_raw, &lon_dir, altitude_raw);

  if (latitude_raw[0] != '\0' && longitude_raw[0] != '\0' &&
      altitude_raw[0] != '\0') {
    data.latitude = convert_to_decimal(latitude_raw, lat_dir);
    data.longitude = convert_to_decimal(longitude_raw, lon_dir);
    data.altitude = atof(altitude_raw);
  }
  return data;
}

void gps_uart_init() {
  // Config uart
  uart_config_t uart_config = {.baud_rate = 9600,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(UART_NUM, &uart_config);
  uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

gps_data_t GPS_get_data() {
  gps_uart_init();
  uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
  gps_data_t gps_data;
  int len =
      uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
  if (len > 0) {
    data[len] = '\0';
    ESP_LOGI(GPS_TAG, "Received: %s", data);
    char *gpgga_ptr = strstr((char *)data, "$GPGGA");
    if (gpgga_ptr) {
      gps_data = parse_gpgga(gpgga_ptr);
      ESP_LOGI(GPS_TAG, "Latitude: %.6f, Longitude: %.6f, Altitude: %.2f m",
               gps_data.latitude, gps_data.longitude, gps_data.altitude);
    }
  }
  free(data);
  return gps_data;
}