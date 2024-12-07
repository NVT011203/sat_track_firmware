#include "sg90.h"

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
  printf("Initializing servo control...\n");
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
}

void test_servo(void *pvParameters) {
  printf("Moving to 0°\n");
  servo_set_angle(MCPWM_UNIT_0, MCPWM_TIMER_0, 0);
}