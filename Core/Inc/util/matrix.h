#ifndef MATRIX_H
#define MATRIX_H

#include <stddef.h>

typedef float mfloat_t;

// Multiply two matrices: A (m x n) and B (n x p) into C (m x p).
// Matrices are stored in row-major order.
void matrix_multiply(const mfloat_t* A, const mfloat_t* B, mfloat_t* C,
                     size_t m, size_t n, size_t p);

// Add two matrices A and B (both m x n) and store the result in C (m x n).
void matrix_add(const mfloat_t* A, const mfloat_t* B, mfloat_t* C,
                size_t m, size_t n);

// Subtract matrix B from matrix A (both m x n) and store the result in C (m x n).
void matrix_subtract(const mfloat_t* A, const mfloat_t* B, mfloat_t* C,
                     size_t m, size_t n);

// Transpose matrix A (m x n) into T (n x m).
void matrix_transpose(const mfloat_t* A, mfloat_t* T,
                      size_t m, size_t n);

// Compute the squared Frobenius norm of matrix A (m x n).
mfloat_t matrix_norm_sq(const mfloat_t* A, size_t m, size_t n);

#endif
