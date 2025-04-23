#include "kalman.h"
#include "util/matrix.h"
#include <stdlib.h>
#include <math.h>

// Helper function to compute the Kalman gain iteratively.
// This function uses the current state transition matrix (stm) stored in kf,
// and the measurement and process noise variances (computed as sigma^2).
static void calculate_gain(KalmanFilter *kf, float alt_sigma, float accel_sigma, float model_sigma)
{
    float alt_variance   = alt_sigma * alt_sigma;
    float accel_variance = accel_sigma * accel_sigma;
    float model_variance = model_sigma * model_sigma;

    // Compute the transpose of the state transition matrix (3x3).
    float stm_t[9];
    matrix_transpose(kf->stm, stm_t, 3, 3);

    // Copy the current gain into last_kgain for convergence checking.
    float last_kgain[6] = { 0 };
    for (int i = 0; i < 6; i++) {
        last_kgain[i] = kf->kgain[i];
    }

    unsigned int iterations = 0;
    // pest: state covariance estimate (3x3); initialize as given.
    float pest[9]  = { 2.0f, 0.0f, 0.0f,
                       0.0f, 9.0f, 0.0f,
                       0.0f, 0.0f, 9.0f };
    float pestp[9] = { 0.0f }; // predicted covariance matrix
    float term[9];              // temporary matrix storage

    while (1)
    {
        // Propagate state covariance:
        // term = stm * pest
        matrix_multiply(kf->stm, pest, term, 3, 3, 3);
        // pestp = term * stm_t
        matrix_multiply(term, stm_t, pestp, 3, 3, 3);
        // Add process noise to the (2,2) element.
        pestp[8] += model_variance;  // (2,2) element in row-major index: 2*3+2 = 8

        // Compute the innovation covariance determinant.
        // Using pestp(0,0), pestp(2,2), pestp(2,0) and pestp(0,2):
        float det = (pestp[0] + alt_variance) * (pestp[8] + accel_variance) - pestp[6] * pestp[2];

        // Update Kalman gain (kgain is 3x2, row-major).
        // Row 0:
        kf->kgain[0] = (pestp[0] * (pestp[8] + accel_variance) - pestp[2] * pestp[6]) / det;
        kf->kgain[1] = (pestp[0] * (-pestp[2]) + pestp[2] * (pestp[0] + alt_variance)) / det;
        // Row 1:
        kf->kgain[2] = (pestp[3] * (pestp[8] + accel_variance) - pestp[5] * pestp[6]) / det;
        kf->kgain[3] = (pestp[3] * (-pestp[2]) + pestp[5] * (pestp[0] + alt_variance)) / det;
        // Row 2:
        kf->kgain[4] = (pestp[6] * (pestp[8] + accel_variance) - pestp[8] * pestp[6]) / det;
        kf->kgain[5] = (pestp[6] * (-pestp[2]) + pestp[8] * (pestp[0] + alt_variance)) / det;

        // Update covariance: pest = f(pestp, kgain)
        // Row 0:
        pest[0] = pestp[0] * (1.0f - kf->kgain[0]) - kf->kgain[1] * pestp[6];
        pest[1] = pestp[1] * (1.0f - kf->kgain[0]) - kf->kgain[1] * pestp[7];
        pest[2] = pestp[2] * (1.0f - kf->kgain[0]) - kf->kgain[1] * pestp[8];
        // Row 1:
        pest[3] = pestp[0] * (-kf->kgain[2]) + pestp[3] - kf->kgain[3] * pestp[6];
        pest[4] = pestp[1] * (-kf->kgain[2]) + pestp[4] - kf->kgain[3] * pestp[7];
        pest[5] = pestp[2] * (-kf->kgain[2]) + pestp[5] - kf->kgain[3] * pestp[8];
        // Row 2:
        pest[6] = (1.0f - kf->kgain[5]) * pestp[6] - kf->kgain[4] * pestp[6];
        pest[7] = (1.0f - kf->kgain[5]) * pestp[7] - kf->kgain[4] * pestp[7];
        pest[8] = (1.0f - kf->kgain[5]) * pestp[8] - kf->kgain[4] * pestp[8];

        iterations++;

        // Check for convergence.
        // Compute the difference norm between current kgain and last_kgain.
        float diff[6];
        for (int i = 0; i < 6; i++) {
            diff[i] = kf->kgain[i] - last_kgain[i];
        }
        float norm_diff = matrix_norm_sq(diff, 1, 6);
        float norm_last = matrix_norm_sq(last_kgain, 1, 6);
        if (norm_last > 0 && (norm_diff / norm_last) < 1e-12f) {
            break;
        }
        // Update last_kgain.
        for (int i = 0; i < 6; i++) {
            last_kgain[i] = kf->kgain[i];
        }
    }
    // (Optionally, you can log the number of iterations for debugging.)
}

void KalmanFilter_init(KalmanFilter *kf, float time_step, float alt_sigma, float accel_sigma, float model_sigma)
{
    // Initialize the state transition matrix (stm) for constant acceleration model.
    // [1, time_step, time_step^2/2;
    //  0, 1,        time_step;
    //  0, 0,        1]
    kf->stm[0] = 1.0f;
    kf->stm[1] = time_step;
    kf->stm[2] = (time_step * time_step) / 2.0f;

    kf->stm[3] = 0.0f;
    kf->stm[4] = 1.0f;
    kf->stm[5] = time_step;

    kf->stm[6] = 0.0f;
    kf->stm[7] = 0.0f;
    kf->stm[8] = 1.0f;

    // Mark that this is the first step.
    kf->first_step = 1;

    // Initialize the state estimates (est and estp) to zero.
    for (int i = 0; i < 3; i++) {
        kf->est[i] = 0.0f;
        kf->estp[i] = 0.0f;
    }

    // Initialize Kalman gain (kgain) to zero.
    for (int i = 0; i < 6; i++) {
        kf->kgain[i] = 0.0f;
    }

    // Calculate the initial Kalman gain using the provided noise parameters.
    calculate_gain(kf, alt_sigma, accel_sigma, model_sigma);
}

void KalmanFilter_step(KalmanFilter *kf, float accel, float altitude)
{
    // On the very first step, use the measured altitude to initialize the position.
    if (kf->first_step) {
        kf->first_step = 0;
        kf->est[0] = altitude;
    }

    // Compute the innovation (measurement residuals):
    // alt_innovation = measured altitude - predicted altitude (first element of estp)
    // accel_innovation = measured acceleration - predicted acceleration (third element of estp)
    float alt_innovation   = altitude - kf->estp[0];
    float accel_innovation = accel - kf->estp[2];

    // Special-case handling for transonic effects.
    // If the altitude error is large and the predicted velocity is in a narrow range,
    // then—if the predicted acceleration is negative—trust the altitude measurement.
    if (fabsf(alt_innovation) > 30.0f && kf->estp[1] > 300.0f && kf->estp[1] < 400.0f) {
        if (kf->estp[2] < 0.0f) {
            kf->est[0] = altitude;
            alt_innovation = 0.0f;
        }
    }

    // Ignore altitude innovations if the measurement is out of range.
    if (altitude > 12000.0f) {
        alt_innovation = 0.0f;
    }

    // Propagate the state: predicted state estp = stm * est.
    // Here, stm is 3x3 and est is 3x1.
    matrix_multiply(kf->stm, kf->est, kf->estp, 3, 3, 1);

    // Form the innovation vector (2x1) from altitude and acceleration innovations.
    float innov[2] = { alt_innovation, accel_innovation };

    // Compute the correction: correction = kgain * innov.
    // kgain is 3x2 and innov is 2x1, yielding a 3x1 correction.
    float correction[3];
    matrix_multiply(kf->kgain, innov, correction, 3, 2, 1);

    // Update the state estimate: est = estp + correction.
    for (int i = 0; i < 3; i++) {
        kf->est[i] = kf->estp[i] + correction[i];
    }

    // (Optional: Insert logging or debugging output here.)
}
