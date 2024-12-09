#ifndef STEP
#define STEP

// Include libraries
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include <math.h>
#include <stdio.h>

// Define pin
#define DIR_PIN GPIO_NUM_32
#define STEP_PIN GPIO_NUM_33

#define MS1_PIN GPIO_NUM_25
#define MS2_PIN GPIO_NUM_26
#define MS3_PIN GPIO_NUM_27

#define STEPS_PER_REV 200
#define MICROSTEPPING 16
#define ANGLE_PER_STEP (360.0 / (STEPS_PER_REV * MICROSTEPPING))

#define RPM 200 // Round per minute
#define STEP_DELAY_US (60 * 1000000 / (RPM * STEPS_PER_REV * MICROSTEPPING))

void rotate_motor(float angle, int direction, int *num_of_steps);
void setup_microstepping_mode();
void stepper_motor_init();

// End
#endif