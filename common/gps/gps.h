#ifndef GPS
#define GPS

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>


#define UART_NUM UART_NUM_2 // UART2
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define BUF_SIZE (1024)
#define GPS_TAG "GPS"

void gps_uart_init();
void gps_task(void *arg);

#endif