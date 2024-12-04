#include "step.h"

void stepper_motor_init() {
  // Cấu hình GPIO
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << DIR_PIN) |
                                           (1ULL << STEP_PIN) |
                                           (1ULL << ENABLE_PIN),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  // Kích hoạt driver
  gpio_set_level(ENABLE_PIN, 0); // 0 để kích hoạt
  setup_microstepping_mode();
}

void setup_microstepping_mode() {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << MS1_PIN) | (1ULL << MS2_PIN) | (1ULL << MS3_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  gpio_set_level(MS1_PIN, 1);
  gpio_set_level(MS2_PIN, 1);
  gpio_set_level(MS3_PIN, 1);
}

void rotate_motor(float angle, int direction) {
  int steps = (int)(angle / ANGLE_PER_STEP); // Tính số bước cần quay
  printf("Rotating %.2f° (%d steps) in direction %d\n", angle, steps,
         direction);

  // Đặt hướng quay
  gpio_set_level(DIR_PIN, direction);

  // Phát xung STEP
  for (int i = 0; i < steps; i++) {
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(STEP_DELAY_US); // Delay giữa các xung
    gpio_set_level(STEP_PIN, 0);
    esp_rom_delay_us(STEP_DELAY_US);
  }
}

void test_step() {
  rotate_motor(90.0, 1); // Quay 90° theo chiều kim đồng hồ
}