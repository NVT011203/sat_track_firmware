#ifndef SG90
#define SG90

// Include libraries
#include "driver/mcpwm.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "soc/mcpwm_periph.h"
#include <stdio.h>


#define SERVO_MIN_PULSEWIDTH 500  // Độ rộng xung tối thiểu (0.5ms) - Góc 0°
#define SERVO_MAX_PULSEWIDTH 2500 // Độ rộng xung tối đa (2.5ms) - Góc 180°
#define SERVO_MAX_DEGREE 180      // Góc quay tối đa của servo

#define SERVO_GPIO 18 // Chân GPIO điều khiển servo

void sg90_init();
void servo_set_angle(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num,
                     uint32_t angle);
uint32_t calculate_pulse_width(uint32_t angle);
void test_servo(void *pvParameters);

// End
#endif