#include "step.h"

void stepper_motor_init() {
  // Config
  gpio_config_t io_conf = {.pin_bit_mask =
                               (1ULL << DIR_PIN) | (1ULL << STEP_PIN),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

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

void rotate_motor(float angle, int direction, int *num_of_steps) {
  int steps = (int)round(angle / ANGLE_PER_STEP);
  printf("Rotating %.2f° (%d steps) in direction %d\n", angle, steps,
         direction);

  // Set direction
  gpio_set_level(DIR_PIN, direction);

  // Rotate
  for (int i = 0; i < steps; i++) {
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(STEP_DELAY_US);
    gpio_set_level(STEP_PIN, 0);
    esp_rom_delay_us(STEP_DELAY_US);
  }
  if (direction == 1) {
    *num_of_steps = *num_of_steps + steps;
  } else {
    *num_of_steps = *num_of_steps - steps;
  }
}