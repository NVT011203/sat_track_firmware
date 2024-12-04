#ifndef EULER_ANGLES
#define EULER_ANGLES

#include <stdio.h>

#define CONFIG_MAGNETIC_DECLINATION -1.8 // Magnetic declination
#define CALC_TAG "CALC"

// Euler angles
typedef struct {
  float roll;
  float pitch;
  float yaw;
} Euler_angles;

// Sensor data type
typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
} Sensor_data;

// Mpu9250 data type
typedef struct {
  Sensor_data accel;
  Sensor_data gyro;
} MPU9250_data;

typedef struct {
  float elevation;
  float azimuth;
} Angles;

// Calc roll, pith, yaw function
Euler_angles Calc_Roll_Pitch_Yaw(Sensor_data *accel, Sensor_data *mag);
// Calc elevation and azimuth
Angles calc_sat_angles(Euler_angles track);

#endif
