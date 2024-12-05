#include "euler_angles.h"

Euler_angles Calc_Roll_Pitch_Yaw(Sensor_data *accel, Sensor_data *mag) {
  Euler_angles angles = {};

  // Calculate roll and pitch
  double roll = atan2(accel->Y, accel->Z);
  double pitch =
      atan2(-accel->X, sqrt(accel->Y * accel->Y + accel->Z * accel->Z));

  // Calculate tilt compensated magnetic field components
  double mag_x_comp = mag->X * cos(pitch) + mag->Z * sin(pitch);
  double mag_y_comp = mag->X * sin(roll) * sin(pitch) + mag->Y * cos(roll) -
                      mag->Z * sin(roll) * cos(pitch);

  // Calculate heading using tilt compensated values
  double heading = atan2(mag_y_comp, mag_x_comp) * (180.0 / M_PI);
  heading += CONFIG_MAGNETIC_DECLINATION; // Adjust heading for local magnetic
                                          // declination

  roll = roll * (180.0 / M_PI);
  pitch = pitch * (180.0 / M_PI);

  angles.roll = roll;
  angles.pitch = pitch;
  angles.yaw = heading;
  return angles;
}

Angles calc_anten_angles(Euler_angles track) {
  double y = track.yaw / (180.0 / M_PI);
  double p = track.pitch / (180.0 / M_PI);
  double r = track.roll / (180.0 / M_PI);
  Angles result = {};
  double R[3][3] = {
      {cos(y) * cos(p), sin(p) * sin(r) * cos(y) - sin(y) * cos(r),
       sin(y) * sin(r) + sin(p) * cos(y) * cos(r)},
      {sin(y) * cos(p), sin(y) * sin(p) * sin(r) + cos(r) * cos(y),
       cos(r) * sin(p) * sin(y) - cos(y) * sin(r)},
      {-sin(p), cos(p) * sin(r), cos(p) * cos(r)}};
  double V[3] = {1, 0, 0};
  double V_1[3];
  for (int i = 0; i < 3; i++) {
    V_1[i] = R[i][0] * V[0] + R[i][1] * V[1] + R[i][2] * V[2];
  }
  // Calc azimuth angle and elevation angle
  double azimuth = atan2(V_1[1], V_1[0]) * (180.0 / M_PI);
  double elevation =
      atan2(V_1[2], sqrt(V_1[0] * V_1[0] + V_1[1] * V_1[1])) * (180.0 / M_PI);
  result.elevation = elevation;
  result.azimuth = azimuth;
  return result;
}

Angles calc_target_angles(LLA_Coordinates track, LLA_Coordinates sat) {
  Angles target_angles;

  double phi_a = track.latitude * M_PI / 180.0;
  double lambda_a = track.longitude * M_PI / 180.0;
  double phi_s = sat.latitude * M_PI / 180.0;
  double lambda_s = sat.longitude * M_PI / 180.0;

  double dphi = phi_s - phi_a;                 // Radian
  double dlambda = lambda_s - lambda_a;        // Radian
  double dh = track.altitude - track.altitude; // Mét

  // ENU
  double dN = EARTH_RADIUS * dphi;
  double dE = EARTH_RADIUS * cos(phi_a) * dlambda;
  double dU = dh;

  target_angles.azimuth = atan2(dE, dN) * (180.0 / M_PI); // Chuyển sang độ
  if (target_angles.azimuth < 0) {
    target_angles.azimuth += 360.0; // Đảm bảo trong khoảng [0, 360]
  }

  target_angles.elevation =
      atan2(dU, sqrt(dE * dE + dN * dN)) * (180.0 / M_PI); // Chuyển sang độ
  return target_angles;
}