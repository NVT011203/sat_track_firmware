#ifndef DATA_TRANSFORM
#define DATA_TRANSFORM

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef char mqtt_data[255];

// Euler angles
// typedef struct {
//   double roll;
//   double pitch;
//   double yaw;
// } Euler_angles;

// // Calc roll, pith, yaw function
// Euler_angles Calc_Roll_Pitch_Yaw(Sensor_data *accel, Sensor_data *mag);
// // Calc elevation and azimuth
// Angles calc_anten_angles(Euler_angles track);
// // Sensor_data ECEF_Transformation(double longitude, double latitude,
// //                                 double altitude);
// Angles calc_target_angles(LLA_Coordinates track, LLA_Coordinates sat);

#endif
