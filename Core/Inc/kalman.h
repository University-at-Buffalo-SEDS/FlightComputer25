#ifndef KALMAN_H
#define KALMAN_H

#include <stdbool.h>

typedef float kfloat_t;

// 3x3 matrix
typedef struct {
    kfloat_t m[3][3];
} Matrix3x3;

// 3x2 matrix
typedef struct {
    kfloat_t m[3][2];
} Matrix3x2;

// 3-element vector
typedef struct {
    kfloat_t v[3];
} Vector3;

// Kalman filter structure
typedef struct {
    Matrix3x3 stm;     // state transition matrix (3x3)
    Matrix3x2 kgain;   // Kalman gain matrix (3x2)
    Vector3 estp;      // predicted state (3x1)
    Vector3 est;       // current state (3x1)
    bool first_step;
} KalmanFilter;

// Initialize the filter with a time step and noise standard deviations.
// alt_sigma: altitude noise standard deviation (m)
// accel_sigma: acceleration noise standard deviation (m/s^2)
// model_sigma: process (model) noise standard deviation (m/s^2)
void KalmanFilter_init(KalmanFilter *kf, kfloat_t time_step,
                       kfloat_t alt_sigma, kfloat_t accel_sigma, kfloat_t model_sigma);

// Alternative initialization with a pre-computed gain matrix.
void KalmanFilter_init_with_gain(KalmanFilter *kf, kfloat_t time_step,
                                 const Matrix3x2 *gain);

// Run one filter update step. 'accel' is acceleration in m/s^2,
// 'altitude' is measured altitude in meters.
void KalmanFilter_step(KalmanFilter *kf, kfloat_t accel, kfloat_t altitude);

// Accessor functions
kfloat_t KalmanFilter_get_pos(const KalmanFilter *kf);
kfloat_t KalmanFilter_get_rate(const KalmanFilter *kf);
kfloat_t KalmanFilter_get_accel(const KalmanFilter *kf);

#endif
