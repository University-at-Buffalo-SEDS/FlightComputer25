#include "kalman.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*-------------------- Helper Functions --------------------*/

// Transpose a 3x3 matrix: out = in^T
static void transpose3x3(const Matrix3x3 *in, Matrix3x3 *out) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out->m[i][j] = in->m[j][i];
}

// Multiply two 3x3 matrices: result = a * b
static void multiply3x3_3x3(const Matrix3x3 *a, const Matrix3x3 *b, Matrix3x3 *result) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result->m[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                result->m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
}

// Multiply a 3x3 matrix by a 3x1 vector: result = a * v
static void multiply3x3_vector(const Matrix3x3 *a, const Vector3 *v, Vector3 *result) {
    for (int i = 0; i < 3; i++) {
        result->v[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            result->v[i] += a->m[i][j] * v->v[j];
        }
    }
}

// Multiply a 3x2 matrix by a 2x1 vector (represented as an array of 2): result = a * vec
static void multiply3x2_vector(const Matrix3x2 *a, const kfloat_t vec[2], Vector3 *result) {
    for (int i = 0; i < 3; i++) {
        result->v[i] = a->m[i][0] * vec[0] + a->m[i][1] * vec[1];
    }
}

// Compute the squared Frobenius norm of a 3x2 matrix.
static kfloat_t norm_sq_matrix3x2(const Matrix3x2 *a) {
    kfloat_t sum = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 2; j++)
            sum += a->m[i][j] * a->m[i][j];
    return sum;
}

// Subtract two 3x2 matrices: result = a - b.
static void subtract_matrix3x2(const Matrix3x2 *a, const Matrix3x2 *b, Matrix3x2 *result) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 2; j++)
            result->m[i][j] = a->m[i][j] - b->m[i][j];
}

// Copy a 3x2 matrix from src to dst.
static void copy_matrix3x2(const Matrix3x2 *src, Matrix3x2 *dst) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 2; j++)
            dst->m[i][j] = src->m[i][j];
}

/*-------------------- Kalman Filter Gain Calculation --------------------*/

// Computes the Kalman gain matrix and updates the error covariance (pest).
// This function iterates until the Kalman gain converges.
static void calculate_gain(KalmanFilter *kf, kfloat_t alt_sigma, kfloat_t accel_sigma, kfloat_t model_sigma) {
    const kfloat_t alt_variance = alt_sigma * alt_sigma;
    const kfloat_t accel_variance = accel_sigma * accel_sigma;
    const kfloat_t model_variance = model_sigma * model_sigma;

    Matrix3x3 stm_t;
    transpose3x3(&kf->stm, &stm_t);

    Matrix3x2 last_kgain = kf->kgain;

    // Initialize error covariance pest (3x3) with given values.
    Matrix3x3 pest = { .m = { {2.0f, 0.0f, 0.0f},
                               {0.0f, 9.0f, 0.0f},
                               {0.0f, 0.0f, 9.0f} } };
    Matrix3x3 pestp = { 0 };
    Matrix3x3 term = { 0 };

    uint16_t iterations = 0;
    const kfloat_t convergence_threshold = 1e-12f;
    while (1) {
        // Propagate state covariance: term = stm * pest, then pestp = term * stm_t
        multiply3x3_3x3(&kf->stm, &pest, &term);
        multiply3x3_3x3(&term, &stm_t, &pestp);
        pestp.m[2][2] += model_variance;

        // Calculate Kalman Gain (kgain = 3x2 matrix)
        kfloat_t det = (pestp.m[0][0] + alt_variance) * (pestp.m[2][2] + accel_variance)
                     - pestp.m[2][0] * pestp.m[0][2];

        kf->kgain.m[0][0] = (pestp.m[0][0] * (pestp.m[2][2] + accel_variance) - pestp.m[0][2] * pestp.m[2][0]) / det;
        kf->kgain.m[0][1] = (pestp.m[0][0] * (-pestp.m[0][2]) + pestp.m[0][2] * (pestp.m[0][0] + alt_variance)) / det;
        kf->kgain.m[1][0] = (pestp.m[1][0] * (pestp.m[2][2] + accel_variance) - pestp.m[1][2] * pestp.m[2][0]) / det;
        kf->kgain.m[1][1] = (pestp.m[1][0] * (-pestp.m[0][2]) + pestp.m[1][2] * (pestp.m[0][0] + alt_variance)) / det;
        kf->kgain.m[2][0] = (pestp.m[2][0] * (pestp.m[2][2] + accel_variance) - pestp.m[2][2] * pestp.m[2][0]) / det;
        kf->kgain.m[2][1] = (pestp.m[2][0] * (-pestp.m[0][2]) + pestp.m[2][2] * (pestp.m[0][0] + alt_variance)) / det;

        // Update error covariance, pest
        Matrix3x3 new_pest;
        new_pest.m[0][0] = pestp.m[0][0] * (1.0f - kf->kgain.m[0][0]) - kf->kgain.m[0][1] * pestp.m[2][0];
        new_pest.m[0][1] = pestp.m[0][1] * (1.0f - kf->kgain.m[0][0]) - kf->kgain.m[0][1] * pestp.m[2][1];
        new_pest.m[0][2] = pestp.m[0][2] * (1.0f - kf->kgain.m[0][0]) - kf->kgain.m[0][1] * pestp.m[2][2];
        new_pest.m[1][0] = pestp.m[0][0] * (-kf->kgain.m[1][0]) + pestp.m[1][0] - kf->kgain.m[1][1] * pestp.m[2][0];
        new_pest.m[1][1] = pestp.m[0][1] * (-kf->kgain.m[1][0]) + pestp.m[1][1] - kf->kgain.m[1][1] * pestp.m[2][1];
        new_pest.m[1][2] = pestp.m[0][2] * (-kf->kgain.m[1][0]) + pestp.m[1][2] - kf->kgain.m[1][1] * pestp.m[2][2];
        new_pest.m[2][0] = (1.0f - kf->kgain.m[2][1]) * pestp.m[2][0] - kf->kgain.m[2][0] * pestp.m[2][0];
        new_pest.m[2][1] = (1.0f - kf->kgain.m[2][1]) * pestp.m[2][1] - kf->kgain.m[2][0] * pestp.m[2][1];
        new_pest.m[2][2] = (1.0f - kf->kgain.m[2][1]) * pestp.m[2][2] - kf->kgain.m[2][0] * pestp.m[2][2];
        pest = new_pest;  // copy new_pest into pest

        // Check for convergence of the Kalman gain.
        Matrix3x2 diff;
        subtract_matrix3x2(&kf->kgain, &last_kgain, &diff);
        kfloat_t diff_norm = norm_sq_matrix3x2(&diff);
        kfloat_t last_norm = norm_sq_matrix3x2(&last_kgain);
        if (last_norm > 0.0f && (diff_norm / last_norm) < convergence_threshold)
            break;

        copy_matrix3x2(&kf->kgain, &last_kgain);
        iterations++;
        if (iterations > 1000) {  // safeguard against infinite loops
            break;
        }
    }

    // Optionally, you can print debugging info here.
    // printf("Kalman gain converged after %u iterations\n", iterations);
}

/*-------------------- Kalman Filter API --------------------*/

void KalmanFilter_init(KalmanFilter *kf, kfloat_t time_step,
                       kfloat_t alt_sigma, kfloat_t accel_sigma, kfloat_t model_sigma) {
    // Set up state transition matrix:
    // stm = [ 1, time_step, time_step^2/2;
    //         0, 1,        time_step;
    //         0, 0,        1 ]
    kf->stm.m[0][0] = 1.0f;
    kf->stm.m[0][1] = time_step;
    kf->stm.m[0][2] = time_step * time_step / 2.0f;
    kf->stm.m[1][0] = 0.0f;
    kf->stm.m[1][1] = 1.0f;
    kf->stm.m[1][2] = time_step;
    kf->stm.m[2][0] = 0.0f;
    kf->stm.m[2][1] = 0.0f;
    kf->stm.m[2][2] = 1.0f;

    // Initialize state vectors to zero.
    for (int i = 0; i < 3; i++) {
        kf->est.v[i] = 0.0f;
        kf->estp.v[i] = 0.0f;
    }
    kf->first_step = true;

    // Calculate the Kalman gain matrix based on the noise parameters.
    calculate_gain(kf, alt_sigma, accel_sigma, model_sigma);
}

void KalmanFilter_init_with_gain(KalmanFilter *kf, kfloat_t time_step,
                                 const Matrix3x2 *gain) {
    // Set up the state transition matrix as above.
    kf->stm.m[0][0] = 1.0f;
    kf->stm.m[0][1] = time_step;
    kf->stm.m[0][2] = time_step * time_step / 2.0f;
    kf->stm.m[1][0] = 0.0f;
    kf->stm.m[1][1] = 1.0f;
    kf->stm.m[1][2] = time_step;
    kf->stm.m[2][0] = 0.0f;
    kf->stm.m[2][1] = 0.0f;
    kf->stm.m[2][2] = 1.0f;

    // Copy the provided gain.
    for (int i = 0; i < 3; i++) {
        kf->kgain.m[i][0] = gain->m[i][0];
        kf->kgain.m[i][1] = gain->m[i][1];
    }

    // Initialize state vectors to zero.
    for (int i = 0; i < 3; i++) {
        kf->est.v[i] = 0.0f;
        kf->estp.v[i] = 0.0f;
    }
    kf->first_step = true;
}

void KalmanFilter_step(KalmanFilter *kf, kfloat_t accel, kfloat_t altitude) {
    // On the first step, initialize the position.
    if (kf->first_step) {
        kf->first_step = false;
        kf->est.v[0] = altitude;
    }

    // Compute innovations.
    kfloat_t alt_innovation = altitude - kf->estp.v[0];
    kfloat_t accel_innovation = accel - kf->estp.v[2];

    // Check for transonic pressure effects and out‐of‐range altitude.
    if (fabsf(alt_innovation) > 30.0f && kf->estp.v[1] > 300.0f && kf->estp.v[1] < 400.0f) {
        if (kf->estp.v[2] < 0.0f) {
            kf->est.v[0] = altitude;
            alt_innovation = 0.0f;
        }
    }
    if (altitude > 12000.0f) {
        alt_innovation = 0.0f;
    }

    // Propagate state: estp = stm * est
    multiply3x3_vector(&kf->stm, &kf->est, &kf->estp);

    // Compute correction = kgain * [alt_innovation; accel_innovation]
    kfloat_t innovation[2] = { alt_innovation, accel_innovation };
    Vector3 correction;
    multiply3x2_vector(&kf->kgain, innovation, &correction);

    // Update state: est = estp + correction
    for (int i = 0; i < 3; i++) {
        kf->est.v[i] = kf->estp.v[i] + correction.v[i];
    }

    // Optionally, add debugging output here.
    // printf("Alt: %.2f, Accel: %.2f, Est Pos: %.2f, Est Rate: %.2f, Est Accel: %.2f\n",
    //        altitude, accel, kf->est.v[0], kf->est.v[1], kf->est.v[2]);
}

kfloat_t KalmanFilter_get_pos(const KalmanFilter *kf) {
    return kf->est.v[0];
}

kfloat_t KalmanFilter_get_rate(const KalmanFilter *kf) {
    return kf->est.v[1];
}

kfloat_t KalmanFilter_get_accel(const KalmanFilter *kf) {
    return kf->est.v[2];
}
