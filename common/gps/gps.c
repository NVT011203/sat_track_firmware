#include "gps.h"

void gps_uart_init()
{
    // Config uart
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void gps_task(void *arg)
{
    gps_uart_init();
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    ESP_LOGI(GPS_TAG, "start");
    while (1)
    {
        // Read data
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            data[len] = '\0';
            ESP_LOGI(GPS_TAG, "Received: %s", data);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    free(data);
}