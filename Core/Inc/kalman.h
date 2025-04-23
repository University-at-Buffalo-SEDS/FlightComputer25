#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

// Kalman filter state structure.
// We assume a 3x1 state vector [position; velocity; acceleration].
// stm is a 3x3 state transition matrix, kgain is a 3x2 Kalman gain,
// estp is the predicted state and est is the updated state.
typedef struct {
    float stm[9];    // 3x3 state transition matrix (row-major)
    float kgain[6];  // 3x2 Kalman gain matrix (row-major)
    float estp[3];   // 3x1 predicted state
    float est[3];    // 3x1 state estimate
    uint8_t first_step;
} KalmanFilter;

// Initialize the Kalman filter given the time step and the noise parameters (standard deviations)
// alt_sigma: altitude measurement noise (m)
// accel_sigma: acceleration measurement noise (m/s^2)
// model_sigma: process/model noise (m/s^2)
void KalmanFilter_init(KalmanFilter *kf, float time_step, float alt_sigma, float accel_sigma, float model_sigma);

// Perform one Kalman filter update step with acceleration (m/s^2) and altitude (m) measurements.
void KalmanFilter_step(KalmanFilter *kf, float accel, float altitude);

#endif
