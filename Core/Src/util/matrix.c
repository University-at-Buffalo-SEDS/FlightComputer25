#include "util/matrix.h"

void matrix_multiply(const mfloat_t* A, const mfloat_t* B, mfloat_t* C,
                     size_t m, size_t n, size_t p)
{
    for (size_t i = 0; i < m; i++) {
        for (size_t j = 0; j < p; j++) {
            C[i * p + j] = 0;
            for (size_t k = 0; k < n; k++) {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

void matrix_add(const mfloat_t* A, const mfloat_t* B, mfloat_t* C,
                size_t m, size_t n)
{
    for (size_t i = 0; i < m * n; i++) {
        C[i] = A[i] + B[i];
    }
}

void matrix_subtract(const mfloat_t* A, const mfloat_t* B, mfloat_t* C,
                     size_t m, size_t n)
{
    for (size_t i = 0; i < m * n; i++) {
        C[i] = A[i] - B[i];
    }
}

void matrix_transpose(const mfloat_t* A, mfloat_t* T,
                      size_t m, size_t n)
{
    for (size_t i = 0; i < m; i++) {
        for (size_t j = 0; j < n; j++) {
            T[j * m + i] = A[i * n + j];
        }
    }
}

mfloat_t matrix_norm_sq(const mfloat_t* A, size_t m, size_t n)
{
    mfloat_t sum = 0;
    for (size_t i = 0; i < m * n; i++) {
        sum += A[i] * A[i];
    }
    return sum;
}
