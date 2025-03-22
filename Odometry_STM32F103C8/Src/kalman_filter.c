#include "kalman_filter.h"
#include "math.h"

static int32_t multiply(int32_t a, int32_t b) {
    return (a * b) / 100;
}

static void matrix_add(int32_t A[3][3], int32_t B[3][3], int32_t result[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result[i][j] = A[i][j] + B[i][j];
}

static void matrix_sub(int32_t A[3][3], int32_t B[3][3], int32_t result[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            result[i][j] = A[i][j] - B[i][j];
}

void kalman_init(KalmanFilter *kf, int32_t dt) {
    kf->state.roll = 0;
    kf->state.pitch = 0;
    kf->state.yaw = 0;

    int32_t init_P[3][3] = {{10000, 0, 0}, {0, 10000, 0}, {0, 0, 10000}};
    int32_t init_Q[3][3] = {{100, 0, 0}, {0, 100, 0}, {0, 0, 100}};
    int32_t init_R[3][3] = {{500, 0, 0}, {0, 500, 0}, {0, 0, 500}};

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = init_P[i][j];
            kf->Q[i][j] = init_Q[i][j];
            kf->R[i][j] = init_R[i][j];
        }

    kf->dt = dt;
}

void kalman_update(KalmanFilter *kf, const SensorData *sensors) {
    kf->state.roll  += multiply(sensors->gyro_x, kf->dt / 1000);
    kf->state.pitch += multiply(sensors->gyro_y, kf->dt / 1000);
    kf->state.yaw   += multiply(sensors->gyro_z, kf->dt / 1000);

    matrix_add(kf->P, kf->Q, kf->P);

    int32_t roll_acc = (int32_t)(atan2(sensors->acc1_y, sensors->acc1_z) * 18000 / 3.14159);
    int32_t pitch_acc = (int32_t)(atan2(-sensors->acc1_x, sensors->acc1_z) * 18000 / 3.14159);
    int32_t yaw_mag = (int32_t)(atan2(sensors->mag_y, sensors->mag_x) * 18000 / 3.14159);

    int32_t z[3] = {roll_acc, pitch_acc, yaw_mag};
    int32_t y[3] = {z[0] - kf->state.roll, z[1] - kf->state.pitch, z[2] - kf->state.yaw};

    int32_t S[3][3];
    matrix_add(kf->P, kf->R, S);

    int32_t K[3];
    for (int i = 0; i < 3; i++)
        K[i] = multiply(kf->P[i][i], 10000 / S[i][i]);

    kf->state.roll  += multiply(K[0], y[0]);
    kf->state.pitch += multiply(K[1], y[1]);
    kf->state.yaw   += multiply(K[2], y[2]);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            kf->P[i][j] = multiply(100 - K[i], kf->P[i][j]) / 100;
}