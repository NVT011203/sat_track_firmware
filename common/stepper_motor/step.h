#ifndef STEP
#define STEP

// Include libraries
#include <stdio.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// Định nghĩa các chân
#define DIR_PIN GPIO_NUM_16
#define STEP_PIN GPIO_NUM_17
#define ENABLE_PIN GPIO_NUM_18

#define MS1_PIN GPIO_NUM_25
#define MS2_PIN GPIO_NUM_26
#define MS3_PIN GPIO_NUM_27

// Định nghĩa các thông số động cơ
#define STEPS_PER_REV 200 // Số bước cho một vòng (Full Step)
#define MICROSTEPPING 16  // Tỷ lệ vi bước (1/16)
#define ANGLE_PER_STEP (360.0 / (STEPS_PER_REV * MICROSTEPPING))

// Định nghĩa các thông số tốc độ
#define RPM 0.1 // Tốc độ (vòng/phút) => 10p / vòng
#define STEP_DELAY_US (60 * 1000000 / (RPM * STEPS_PER_REV * MICROSTEPPING))

void rotate_motor(float angle, int direction);
void setup_microstepping_mode();
void stepper_motor_init();

// End
#endif