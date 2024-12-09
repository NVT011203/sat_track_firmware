// C libraries and idf libraries
#include "driver/gptimer.h"
#include "driver/mcpwm.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "soc/mcpwm_periph.h"
#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/queue.h>
#include <FreeRTOS/semphr.h>
#include <FreeRTOS/task.h>
#include <stdbool.h>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_sntp.h"
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
#include "trans.h"

// #include "kalman.h"

static const char *TAG = "MAIN";
static const char *MQTT_TAG = "MQTT";
static const char *WIFI_TAG = "WIFI";
static const char *TIMER_TAG = "GPTimer";
static const char *SG90_TAG = "SG90";
static const char *SET_SAT_TAG = "Set Satellite";
static const char *TIME_TAG = "Time";
#define WIFI_SSID "NVT"      // Wifi ssid
#define WIFI_PASS "12345678" // Wifi password
#define RECEIVE_TOPIC "/topic/tle"
#define SEND_TOPIC "/topic/sat_data"
#define UPDATE_ANGLES_TOPIC "/topic/update_angles"
#define CHECK_TRACK "/topic/check_track"
#define CHECK_SERVER "/topic/check_server"
#define SET_SAT "/topic/set_sat"

#define SERVO_MIN_PULSEWIDTH 500  // Độ rộng xung tối thiểu (0.5ms) - Góc 0°
#define SERVO_MAX_PULSEWIDTH 2500 // Độ rộng xung tối đa (2.5ms) - Góc 180°
#define SERVO_MAX_DEGREE 180      // Góc quay tối đa của servo
#define SERVO_GPIO 13             // Chân GPIO điều khiển servo

#define MQTT_CORE_ID 1 // Core 1

// Global variable start
esp_mqtt_client_handle_t client;
SemaphoreHandle_t mqttMutex;
QueueHandle_t set_sat_queue;
QueueHandle_t update_angles_queue;
esp_timer_handle_t periodic_timer;
bool set_sat = false;
bool is_updating = false;
Sat_data_angles sat_angles[34];
Satellite_data start_angles = {};
double sg90_angle_declination = 0;
int steps = 0;
time_t now;
struct tm timeinfo;
// End

// Wifi start
const int WIFI_CONNECTED_BIT = BIT0;
const int SET_SAT_BIT = BIT1;
static EventGroupHandle_t event_group;
static EventGroupHandle_t wifi_event_group;
void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
void wifi_init_sta();
// Wifi end

// Servo sg90
void sg90_init();
void servo_set_angle(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num,
                     uint32_t angle);
uint32_t calculate_pulse_width(uint32_t angle);
// Servo end

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

// Set up satellite
void set_satellite(sat_data_set message);

// Timer start
// Timer callback function
void timer_callback(void *arg);
void timer_init();
// Timer end

// Set time
void initialize_sntp();

// I2C init
void i2c_init();

// Read data sensor functions
Euler_angles get_sen_data();

// Calculate declination angle and reset
void sg90_reset_angle();

// Task function
void mqtt_task(void *pvParameters);
// App main
void app_main(void) {
  // Create queue
  set_sat_queue = xQueueCreate(1, sizeof(sat_data_set));
  update_angles_queue = xQueueCreate(1, sizeof(sat_data_set));
  // Create group
  event_group = xEventGroupCreate();
  wifi_event_group = xEventGroupCreate();
  mqttMutex = xSemaphoreCreateMutex();
  // Config mqtt
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
  // Init wifi
  esp_netif_init();
  esp_event_loop_create_default();
  ESP_LOGI(WIFI_TAG, "Wifi initializing...");
  wifi_init_sta();
  // Waitting for connection
  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                      portMAX_DELAY);

  // I2c init
  i2c_init();
  // Qmc5883l init
  // qmc5883l_init();
  // Init rotator
  sg90_init();
  stepper_motor_init();
  // Reset angles
  sg90_reset_angle();

  // Start timer
  timer_init();
  // Set time
  initialize_sntp();
  // Mqtt task in core 2
  xTaskCreatePinnedToCore(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL,
                          MQTT_CORE_ID);
  // xTaskCreate(get_time, "get_time", 2048, NULL, 10, NULL);
}

// Set up satellite
void set_satellite(sat_data_set message) {
  if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
    esp_timer_stop(periodic_timer);
    ESP_LOGI(TIMER_TAG, "Periodic timer stopped");
    ESP_LOGI(SET_SAT_TAG, "Set new satellite...");
    Satellite_data sat_data = data_trans(message, sat_angles);
    start_angles.time_to_start = sat_data.time_to_start;
    start_angles.start_elevation = sat_data.start_elevation;
    start_angles.start_azimuth = sat_data.start_azimuth;

    printf("Time to start: %d\n", start_angles.time_to_start);
    printf("Start elevation: %f\n", start_angles.start_elevation);
    printf("Start azimuth: %f\n", start_angles.start_azimuth);

    for (int i = 0; i < 34; i++) {
      printf("Time: %d\n", sat_angles[i].time);
      printf("Elevation: %f\n", sat_angles[i].elevation);
      printf("Azimuth: %f\n", sat_angles[i].azimuth);
    }

    ESP_LOGI(SET_SAT_TAG, "OK");
    ESP_LOGI(TIMER_TAG, "Periodic timer started, 1s interval");
    xSemaphoreGive(mqttMutex);
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000));
  }
}

// Timer start
void timer_callback(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  time(&now);
  localtime_r(&now, &timeinfo);
  Sat_data_angles data_angle;
  if (set_sat) {
    if (xSemaphoreTakeFromISR(mqttMutex, &xHigherPriorityTaskWoken) == pdTRUE) {
      int status = find_angles(now, sat_angles, &data_angle);
      if (status == 1 || status == 2) {
        printf("Found data in time: %d\n", data_angle.time);
        if (is_updating && status == 2) {
          sat_data_set update_sat_message;
          xQueueReceiveFromISR(update_angles_queue, update_sat_message,
                               &xHigherPriorityTaskWoken);
          Satellite_data sat_data = data_trans(update_sat_message, sat_angles);

          for (int i = 0; i < 34; i++) {
            printf("Time: %d\n", sat_angles[i].time);
            printf("Elevation: %f\n", sat_angles[i].elevation);
            printf("Azimuth: %f\n", sat_angles[i].azimuth);
          }

          ESP_LOGI(SET_SAT_TAG, "Satellite angles updated!!!!!!!!!!!!!!!!!!!");
          is_updating = false;
          set_sat = true;
          if (is_updating) {
            ESP_LOGI(TAG, "Please update new angles!");
          }
        }
        Euler_angles angles = get_sen_data();
        int time = (int)round(esp_timer_get_time() / 1000000);
        ESP_LOGI(TAG, "Time: %d", time);
        if (abs((int)round(angles.pitch - data_angle.elevation)) >= 1) {
          servo_set_angle(MCPWM_UNIT_0, MCPWM_TIMER_0,
                          (int)(data_angle.elevation - sg90_angle_declination));
        }
        if (steps * ANGLE_PER_STEP - data_angle.azimuth > ANGLE_PER_STEP) {
          rotate_motor(
              fabs((float)(steps * ANGLE_PER_STEP - data_angle.azimuth)), 0,
              &steps);
        } else if (steps * ANGLE_PER_STEP - data_angle.azimuth <
                   -ANGLE_PER_STEP) {
          rotate_motor(
              fabs((float)(-steps * ANGLE_PER_STEP + data_angle.azimuth)), 1,
              &steps);
        }
        ESP_LOGI(TAG, "Elevation: %lf, Azimuth: %lf", angles.pitch, angles.yaw);
        xSemaphoreGiveFromISR(mqttMutex, &xHigherPriorityTaskWoken);
      } else if (status == 0) {
        servo_set_angle(MCPWM_UNIT_0, MCPWM_TIMER_0, 0);
        rotate_motor(fabs((float)(steps * ANGLE_PER_STEP)), 0, &steps);
        set_sat = false;
        printf("Set set_sat = false????????????????????????");
        esp_timer_stop(periodic_timer);
      }
    }
  } else {
    ESP_LOGI(TIMER_TAG, "Please set satellite!");
    printf("Set sat: %d\n", set_sat);
  }
}

// Timer start
void timer_init() {
  esp_timer_create_args_t timer_config = {.callback = &timer_callback,
                                          .arg = NULL,
                                          .dispatch_method = ESP_TIMER_TASK,
                                          .name = "periodic_timer"};

  ESP_ERROR_CHECK(esp_timer_create(&timer_config, &periodic_timer));
}
// Timer end

// Set time
void initialize_sntp() {
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
}

// I2C init
void i2c_init() {
  // Init i2c and connect sensors
  esp_err_t ret = i2c_master_init();
  if (ret != ESP_OK) {
    ESP_LOGE(I2C_TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
    return;
  }
}

// Get sensors data
Euler_angles get_sen_data() {
  Sensor_data accel = {};
  Sensor_data gyro = {};

  Euler_angles angles;
  // int16_t x, y, z;
  // Mqtt message
  // mqtt_data message;

  // Read data
  mpu9250_read_sensor_data(&accel, &gyro);
  // qmc5883l_read_magnetometer(&x, &y, &z);
  // Handle data
  // angles = Calc_Roll_Pitch_Yaw(&accel, &(Sensor_data){x, y, z});
  angles = Calc_Roll_Pitch_Yaw(&accel);
  ESP_LOGI(CALC_TAG, "Eulers angels roll: %lf, pitch: %lf, yaw: %lf",
           angles.roll, angles.pitch, angles.yaw);
  // snprintf(message, sizeof(message), "{Elevation: %lf, Azimuth: %lf}",
  //          result.elevation, result.azimuth);
  angles.pitch = -angles.pitch;
  return angles;
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
    xEventGroupClearBits(event_group, WIFI_CONNECTED_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(WIFI_TAG, "Connected, IP address: " IPSTR,
             IP2STR(&event->ip_info.ip));

    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta() {
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
  sat_data_set message;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    // Subcribe receive topic
    msg_id = esp_mqtt_client_subscribe(client, RECEIVE_TOPIC, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    // Subcribe send topic
    msg_id = esp_mqtt_client_subscribe(client, SEND_TOPIC, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    // Subcribe satellite angles topic
    msg_id = esp_mqtt_client_subscribe(client, UPDATE_ANGLES_TOPIC, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    // Subcribe check tracker topic
    msg_id = esp_mqtt_client_subscribe(client, CHECK_TRACK, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    // Subcribe check server topic
    msg_id = esp_mqtt_client_subscribe(client, CHECK_SERVER, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    // Subcribe set satellite topic
    msg_id = esp_mqtt_client_subscribe(client, SET_SAT, 0);
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
    if (memcmp(event->topic, RECEIVE_TOPIC, event->topic_len) == 0) {
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
    } else if (memcmp(event->topic, UPDATE_ANGLES_TOPIC, event->topic_len) ==
               0) {
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
      memcpy(message, event->data, event->data_len);
      xQueueSend(update_angles_queue, message, portMAX_DELAY);
      is_updating = true;
    } else if (memcmp(event->topic, CHECK_TRACK, event->topic_len) == 0) {
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
    } else if (memcmp(event->topic, CHECK_SERVER, event->topic_len) == 0) {
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
    } else if (memcmp(event->topic, SET_SAT, event->topic_len) == 0) {
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
      memcpy(message, event->data, event->data_len);
      xQueueSend(set_sat_queue, message, portMAX_DELAY);
      xEventGroupSetBits(event_group, SET_SAT_BIT);
    }
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

      .credentials.username = "abc123456",
      .credentials.authentication.password = "abc123456T",
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

void mqtt_task(void *pvParameters) {
  // Mqtt start
  mqtt_app_start();
  // int msg_id;
  // mqtt_data message;
  while (1) {
    xEventGroupWaitBits(event_group, SET_SAT_BIT, pdFALSE, pdTRUE,
                        portMAX_DELAY);
    sat_data_set set_sat_message;
    xQueueReceive(set_sat_queue, set_sat_message, pdMS_TO_TICKS(portMAX_DELAY));
    set_satellite(set_sat_message);
    set_sat = true;
    ESP_LOGI(MQTT_TAG, "Set satellite");
    xEventGroupClearBits(event_group, SET_SAT_BIT);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// MQTT end

// Rotator
// Calculate pusle width from angle
static uint32_t calculate_pulse_width(uint32_t angle) {
  uint32_t pulse_width =
      SERVO_MIN_PULSEWIDTH +
      ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) /
          SERVO_MAX_DEGREE;
  return pulse_width;
}

// Set angle for servo
static void servo_set_angle(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num,
                            uint32_t angle) {
  uint32_t pulse_width = calculate_pulse_width(angle);
  mcpwm_set_duty_in_us(mcpwm_num, timer_num, MCPWM_OPR_A, pulse_width);
}

void sg90_init() {
  // Config mcpwm
  ESP_LOGI(SG90_TAG, "Initializing servo control...");
  // Config mcpwm and gpio
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO);

  mcpwm_config_t pwm_config = {
      .frequency = 50,
      .cmpr_a = 0,
      .cmpr_b = 0,
      .duty_mode = MCPWM_DUTY_MODE_0,
      .counter_mode = MCPWM_UP_COUNTER,
  };

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  ESP_LOGI(SG90_TAG, "Initialized");
}
// Rotator end

// Calculate declination angle and reset
void sg90_reset_angle() {
  servo_set_angle(MCPWM_UNIT_0, MCPWM_TIMER_0, 0);
  vTaskDelay(pdMS_TO_TICKS(500));
  Euler_angles angle_1 = get_sen_data();
  vTaskDelay(pdMS_TO_TICKS(200));
  Euler_angles angle_2 = get_sen_data();
  sg90_angle_declination = (angle_1.pitch + angle_2.pitch) / 2;
}