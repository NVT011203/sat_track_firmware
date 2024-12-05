// C libraries and idf libraries
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/queue.h>
#include <FreeRTOS/task.h>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_tls.h"
#include "mqtt_client.h"
#include <sys/param.h>

// Customs libraries
#include "euler_angles.h"
#include "gps.h"
#include "i2c_init.h"
#include "mpu9250.h"
#include "qmc5883l.h"
#include "step.h"
// #include "kalman.h"

static const char *TAG = "MAIN";
static const char *MQTT_TAG = "MQTT";
static const char *WIFI_TAG = "WIFI";
#define WIFI_SSID "NVT"      // Wifi ssid
#define WIFI_PASS "12345678" // Wifi password
#define RECEIVE_TOPIC "/topic/tle"
#define SEND_TOPIC "/topic/sat_data"
typedef char mqtt_data[100];

// Start global variable
esp_mqtt_client_handle_t client;
QueueHandle_t test_queue;

//

// Wifi start
const int WIFI_CONNECTED_BIT = BIT0;
static EventGroupHandle_t wifi_event_group;
void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
void wifi_init_sta();
// Wifi end

// Load certificate
#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipseprojects_io_pem_start[] =
    "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE
    "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_eclipseprojects_io_pem_start[] asm(
    "_binary_mqtt_eclipseprojects_io_pem_start");
#endif
extern const uint8_t mqtt_eclipseprojects_io_pem_end[] asm(
    "_binary_mqtt_eclipseprojects_io_pem_end");

// MQTT start
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);
static void mqtt_app_start(void);
// MQTT end

// Task function
void Get_data_task(void *pvParameters);

void Test_task(void *pvParameters) {
  while (1) {
    ESP_LOGI(TAG, "Test task is running!");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void send_mqtt_task(void *pvParameters) {
  int msg_id;
  mqtt_data message;
  while (1) {
    if (xQueueReceive(test_queue, &message, portMAX_DELAY) == pdPASS) {
      msg_id = esp_mqtt_client_publish(client, SEND_TOPIC, message, 0, 0, 0);
      ESP_LOGI(TAG, "Message id %d sent!", msg_id);
    }
  }
}

// App main
void app_main(void) {
  test_queue = xQueueCreate(2, 100);
  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes",
           esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
  esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
  esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
  esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
  esp_log_level_set("transport", ESP_LOG_VERBOSE);
  esp_log_level_set("outbox", ESP_LOG_VERBOSE);

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  esp_netif_init();
  esp_event_loop_create_default();
  ESP_LOGI(WIFI_TAG, "Wifi initializing...");
  wifi_init_sta();

  // Waiting for connection
  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                      portMAX_DELAY);

  mqtt_app_start();

  // xTaskCreate(Wifi_connection_task, "Wifi_station_init", 4096, NULL, 10,
  // NULL); xTaskCreate(Test_task, "Test_task", 4096, NULL, 2, NULL);
  xTaskCreate(Get_data_task, "Sensor_read_data", 4096, NULL, 2, NULL);
  // xTaskCreate(send_mqtt_task, "Send_mqtt", 4096, NULL, 2, NULL);
}

// Get sensors data
void Get_data_task(void *pvParameters) {
  esp_err_t ret = i2c_master_init();
  if (ret != ESP_OK) {
    ESP_LOGE(I2C_TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
    return;
  }
  // Qmc5883l init
  qmc5883l_init();
  ESP_LOGI(QMC_TAG, "QMC5883L initialized");
  Sensor_data accel = {};
  Sensor_data gyro = {};

  Euler_angles angles;
  int16_t x, y, z;
  // Mqtt message
  mqtt_data message;

  while (1) {
    // Read data
    mpu9250_read_sensor_data(&accel, &gyro);
    qmc5883l_read_magnetometer(&x, &y, &z);
    // Handle data
    angles = Calc_Roll_Pitch_Yaw(&accel, &(Sensor_data){x, y, z});
    ESP_LOGI(CALC_TAG, "mag-X: %d, mag-Y: %d, mag-Z: %d", x, y, z);
    // ESP_LOGI(CALC_TAG, "yaw__: %lf", yaw__);
    ESP_LOGI(CALC_TAG, "Eulers angels roll: %lf, pitch: %lf, yaw: %lf",
             angles.roll, angles.pitch, angles.yaw);
    Angles result = calc_anten_angles(angles);
    ESP_LOGI(CALC_TAG, ": Elevation: %lf, Azimuth: %lf", result.elevation,
             result.azimuth);
    snprintf(message, sizeof(message), "{Elevation: %lf, Azimuth: %lf}",
             result.elevation, result.azimuth);
    if (xQueueSend(test_queue, &message, pdMS_TO_TICKS(500)) == pdPASS) {
      printf("Sent to test queue!\n");
    };
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Wifi start
void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGI(WIFI_TAG, "Wifi disconected, reconnecting...");
    esp_wifi_connect();
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(WIFI_TAG, "Connected, IP address: " IPSTR,
             IP2STR(&event->ip_info.ip));

    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta() {
  wifi_event_group = xEventGroupCreate();
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(WIFI_TAG, "WiFi STA Initialized.");
}
// Wifi end

// MQTT start
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32,
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    msg_id = esp_mqtt_client_subscribe(client, RECEIVE_TOPIC, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

    msg_id = esp_mqtt_client_subscribe(client, SEND_TOPIC, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    msg_id = esp_mqtt_client_publish(client, SEND_TOPIC, "Tracker is running!",
                                     0, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

static void mqtt_app_start(void) {
  const esp_mqtt_client_config_t mqtt_cfg = {
      .broker =
          {.address.uri =
               "mqtts://aee1fa99e37f41f0afcf1015de05aad1.s1.eu.hivemq.cloud",
           .address.port = 8883,
           .verification.certificate =
               (const char *)mqtt_eclipseprojects_io_pem_start},

      .credentials.username = "trangnv",
      .credentials.authentication.password = "T0101t2003",
      .session.protocol_ver = MQTT_PROTOCOL_V_5,
  };

  ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes",
           esp_get_free_heap_size());
  client = esp_mqtt_client_init(&mqtt_cfg);
  /* The last argument may be used to pass data to the event handler, in this
   * example mqtt_event_handler */
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);
  esp_mqtt_client_start(client);
}
// MQTT end