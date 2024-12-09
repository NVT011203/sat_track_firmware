#ifndef DATA_TRANSFORM
#define DATA_TRANSFORM

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "euler_angles.h"

typedef char mqtt_data[255];
typedef char sat_data_set[1000];

typedef struct {
  int time;
  float elevation;
  float azimuth;
} Sat_data_angles;

// Satellite data
typedef struct {
  float start_azimuth;
  float start_elevation;
  int time_to_start;
} Satellite_data;

Satellite_data data_trans(sat_data_set mqtt_data_receive,
                          Sat_data_angles *angles);
void update_sat_angles(sat_data_set mqtt_data_receive, Sat_data_angles *angles);
int find_angles(long long int time, Sat_data_angles *data_angles,
                Sat_data_angles *data_angle);

#endif
