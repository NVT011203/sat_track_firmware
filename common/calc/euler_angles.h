#ifndef EULER_ANGLES
#define EULER_ANGLES

#include <math.h>
#include <stdio.h>

#define CONFIG_MAGNETIC_DECLINATION -1.8 // Magnetic declination
#define CALC_TAG "CALC"
#define EARTH_RADIUS 6378137.0

// Euler angles
typedef struct {
  double roll;
  double pitch;
  double yaw;
} Euler_angles;

// Sensor data type
typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
} Sensor_data;

typedef struct {
  double longitude;
  double latitude;
  double altitude;
} LLA_Coordinates;

// Mpu9250 data type
typedef struct {
  Sensor_data accel;
  Sensor_data gyro;
} MPU9250_data;

typedef struct {
  double elevation;
  double azimuth;
} Angles;

// Calc roll, pith, yaw
// Euler_angles Calc_Roll_Pitch_Yaw(Sensor_data *accel, Sensor_data *mag);
Euler_angles Calc_Roll_Pitch_Yaw(Sensor_data *accel);
// Calc elevation and azimuth
// Angles calc_anten_angles(Euler_angles track);
// Sensor_data ECEF_Transformation(double longitude, double latitude,
//                                 double altitude);
Angles calc_target_angles(LLA_Coordinates track, LLA_Coordinates sat);

#endif
