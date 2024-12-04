#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include <stdio.h>
#include <string.h>

typedef struct
{
    float x[3];    // Trạng thái [roll, pitch, yaw]
    float P[3][3]; // Ma trận hiệp phương sai
    float Q[3][3]; // Nhiễu mô hình
    float R[3][3]; // Nhiễu đo lường
    float F[3][3]; // Ma trận chuyển đổi trạng thái
    float H[3][3]; // Ma trận đo lường
} KalmanFilter;

void kalman_init(KalmanFilter *kf);
void kalman_predict(KalmanFilter *kf, float gyro_inputs[3], float dt);
void kalman_update(KalmanFilter *kf, float measurements[3]);
void process_kalman(KalmanFilter *kf, float acc[3], float gyro[3], float mag[3], float angles[3]);

#endif