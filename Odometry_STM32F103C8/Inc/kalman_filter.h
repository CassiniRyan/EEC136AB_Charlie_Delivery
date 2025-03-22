#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

typedef struct {
    int16_t acc1_x, acc1_y, acc1_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t acc2_x, acc2_y, acc2_z;
    int16_t mag_x, mag_y, mag_z;
} SensorData;

typedef struct {
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
} KalmanState;

typedef struct {
    KalmanState state;
    int32_t P[3][3];
    int32_t Q[3][3];
    int32_t R[3][3];
    int32_t dt;
} KalmanFilter;

void kalman_init(KalmanFilter *kf, int32_t dt);
void kalman_update(KalmanFilter *kf, const SensorData *sensors);

#endif // KALMAN_FILTER_H