#include "kalman.h"

void kalman_init(KalmanFilter *kf)
{
    memset(kf, 0, sizeof(KalmanFilter));

    // Khởi tạo ma trận hiệp phương sai và nhiễu
    for (int i = 0; i < 3; i++)
    {
        kf->P[i][i] = 1.0;  // Hiệp phương sai ban đầu
        kf->Q[i][i] = 0.01; // Nhiễu mô hình
        kf->R[i][i] = 0.1;  // Nhiễu đo lường
        kf->F[i][i] = 1.0;  // Trạng thái không thay đổi
        kf->H[i][i] = 1.0;  // Liên kết đo lường
    }
}

void kalman_predict(KalmanFilter *kf, float gyro_inputs[3], float dt)
{
    // Dự đoán trạng thái mới: x_k = F * x + gyro_inputs * dt
    for (int i = 0; i < 3; i++)
    {
        kf->x[i] += gyro_inputs[i] * dt;
    }

    // Dự đoán ma trận hiệp phương sai: P_k = F * P * F' + Q
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            kf->P[i][j] += kf->Q[i][j] * dt;
        }
    }
}

void kalman_update(KalmanFilter *kf, float measurements[3])
{
    float y[3]; // Sai số đo lường: y = z - H * x
    float S[3][3], K[3][3], P_temp[3][3];

    // Tính toán sai số đo lường
    for (int i = 0; i < 3; i++)
    {
        y[i] = measurements[i] - kf->x[i];
    }

    // Tính toán Kalman Gain: K = P * H' * (H * P * H' + R)^-1
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            S[i][j] = kf->P[i][j] + kf->R[i][j];
            K[i][j] = kf->P[i][j] / S[i][j];
        }
    }

    // Cập nhật trạng thái: x = x + K * y
    for (int i = 0; i < 3; i++)
    {
        kf->x[i] += K[i][i] * y[i];
    }

    // Cập nhật ma trận hiệp phương sai: P = (I - K * H) * P
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            P_temp[i][j] = kf->P[i][j] - K[i][j] * kf->P[j][j];
        }
    }
    memcpy(kf->P, P_temp, sizeof(P_temp));
}

void process_kalman(KalmanFilter *kf, float acc[3], float gyro[3], float mag[3], float angles[3])
{
    // Dự đoán
    kalman_predict(kf, gyro, 0.01); // Chu kỳ 10ms

    // Cập nhật
    kalman_update(kf, angles);

    printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", kf->x[0], kf->x[1], kf->x[2]);
}
