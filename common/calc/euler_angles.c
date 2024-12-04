#include "euler_angles.h"
#include <math.h>

Euler_angles Calc_Roll_Pitch_Yaw(Sensor_data *accel, Sensor_data *mag) {
  Euler_angles angles = {};

  // Calculate roll and pitch
  float roll = atan2(accel->Y, accel->Z);
  float pitch =
      atan2(-accel->X, sqrt(accel->Y * accel->Y + accel->Z * accel->Z));

  // Calculate tilt compensated magnetic field components
  float mag_x_comp = mag->X * cos(pitch) + mag->Z * sin(pitch);
  float mag_y_comp = mag->X * sin(roll) * sin(pitch) + mag->Y * cos(roll) -
                     mag->Z * sin(roll) * cos(pitch);

  // Calculate heading using tilt compensated values
  float heading = atan2(mag_y_comp, mag_x_comp) * (180.0 / M_PI);
  heading += CONFIG_MAGNETIC_DECLINATION; // Adjust heading for local magnetic
                                          // declination

  roll = roll * (180.0 / M_PI);
  pitch = pitch * (180.0 / M_PI);

  angles.roll = roll;
  angles.pitch = pitch;
  angles.yaw = heading;
  return angles;
}

Angles calc_sat_angles(Euler_angles track) {
  float y = track.yaw / (180.0 / M_PI);
  float p = track.pitch / (180.0 / M_PI);
  float r = track.roll / (180.0 / M_PI);
  Angles result = {};
  float R[3][3] = {{cos(y) * cos(p), sin(p) * sin(r) * cos(y) - sin(y) * cos(r),
                    sin(y) * sin(r) + sin(p) * cos(y) * cos(r)},
                   {sin(y) * cos(p), sin(y) * sin(p) * sin(r) + cos(r) * cos(y),
                    cos(r) * sin(p) * sin(y) - cos(y) * sin(r)},
                   {-sin(p), cos(p) * sin(r), cos(p) * cos(r)}};
  float V[3] = {1, 0, 0};
  float V_1[3];
  for (int i = 0; i < 3; i++) {
    V_1[i] = R[i][0] * V[0] + R[i][1] * V[1] + R[i][2] * V[2];
  }
  // Calc azimuth angle and elevation angle
  float azimuth = atan2(V_1[1], V_1[0]) * (180.0 / M_PI);
  float elevation =
      atan2(V_1[2], sqrt(V_1[0] * V_1[0] + V_1[1] * V_1[1])) * (180.0 / M_PI);
  result.elevation = elevation;
  result.azimuth = azimuth;
  return result;
}
