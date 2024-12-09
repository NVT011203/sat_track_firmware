#include "trans.h"

Satellite_data data_trans(sat_data_set mqtt_data_receive,
                          Sat_data_angles *angles) {
  Satellite_data start_angles;
  char *token;
  token = strtok(mqtt_data_receive, " ,"); // Tách theo dấu cách và dấu phẩy
  int count = 0;
  int el = 0;
  while (token != NULL) {
    if (el == 0) {
      if (count == 0) {
        start_angles.time_to_start = atoi(token);
      }
      angles[count].time = atoi(token);
    } else if (el == 1) {
      if (count == 0) {
        start_angles.start_elevation = atof(token);
      }
      angles[count].elevation = atof(token);
    } else {
      if (count == 0) {
        start_angles.start_azimuth = atof(token);
      }
      angles[count].azimuth = atof(token);
    }
    el++;
    if (el == 3) {
      el = 0;
      count++;
    }
    token = strtok(NULL, " ,");
  }
  return start_angles;
}

int find_angles(long long int time, Sat_data_angles *data_angles,
                Sat_data_angles *data_angle) {
  for (int i = 0; i < 34; i++) {
    if (data_angles[i].time == time) {
      *data_angle = data_angles[i];
      if (i >= 32) {
        return 2;
      }
      return 1;
    }
  }
  return 0;
}