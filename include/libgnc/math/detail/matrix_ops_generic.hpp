#pragma once

#include <cmath>

#include "libgnc/assert.hpp"

#include "matrix_fwd.hpp"
#include "vector_fwd.hpp"

namespace gnc::detail
{
    // matrix x matrix
    template<typename T, std::size_t Rows, std::size_t Cols, std::size_t OtherCols>
    constexpr Matrix<T, Rows, OtherCols> matmul_impl(const Matrix<T, Rows, Cols>& lhs, const Matrix<T, Cols, OtherCols>& rhs) {
        Matrix<T, Rows, OtherCols> result;

        for(std::size_t r=0; r<Rows; ++r) {
            for(std::size_t c=0; c<OtherCols; ++c) {
                T sum{};

                for(std::size_t k=0; k<Cols; ++k)
                    sum += lhs(r, k) * rhs(k, c);

                result(r, c) = sum;
            }
        }

        return result;
    }

    // matrix x vector
    template<typename T, std::size_t Rows, std::size_t Cols>
    constexpr Vec<T, Rows> matvec_impl(const Matrix<T, Rows, Cols>& mat, const Vec<T, Cols>& vec) {
        Vec<T, Rows> result;

        for(std::size_t r=0; r<Rows; ++r) {
            T sum{};

            for(std::size_t c=0; c<Cols; ++c)
                sum += mat(r, c) * vec[c];

            result[r] = sum;
        }

        return result;
    }

    // transpose
    template<typename T, std::size_t Rows, std::size_t Cols>
    constexpr Matrix<T, Cols, Rows> transpose_impl(const Matrix<T, Rows, Cols>& mat) {
        Matrix<T, Cols, Rows> result;

        for(std::size_t r= 0; r<Rows; ++r) {
            for(std::size_t c=0; c<Cols; ++c) {
                result(c, r) = mat(r, c);
            }
        }

        return result;
    }

    // trace
    template<typename T, std::size_t N>
    constexpr T trace_impl(const Matrix<T, N, N>& mat) {
        T result{};

        for(std::size_t i=0; i<N; ++i)
            result += mat(i, i);

        return result;
    }

    // determinant (Laplace)
    template<typename T>
    constexpr T determinant_impl(const Matrix<T, 1, 1>& mat) {
        return mat(0, 0);
    }

    template<typename T, std::size_t N>
    constexpr T determinant_impl(const Matrix<T, N, N>& mat) {
        T det{};

        for(std::size_t col=0; col<N; ++col) {
            Matrix<T, N-1, N-1> minor;

            for(std::size_t r=1; r<N; ++r) {
                std::size_t mc=0;

                for(std::size_t c=0; c<N; ++c) {
                    if(c == col) continue;

                    minor(r-1, mc++) = mat(r, c);
                }
            }

            const T sign = (col % 2 == 0) ? T(1) : T(-1);

            det += sign * mat(0, col) * determinant_impl(minor);
        }

        return det;
    }

    // inverse (Gauss-Jordan)
    template<typename T, std::size_t N>
    constexpr Matrix<T, N, N> inverse_impl(const Matrix<T, N, N>& mat) {
        Matrix<T, N, N> a = mat;
        Matrix<T, N, N> inv;

        for(std::size_t i=0; i<N; ++i)
            inv(i, i) = T(1);

        for(std::size_t i=0; i<N; ++i) {
            T pivot = a(i, i);

            // check if singular matrix
            LIBGNC_ASSERT(std::abs(pivot) >= T(1e-8) && "Trying to invert a singular matrix");

            const T inv_pivot = T(1) / pivot;

            for(std::size_t j=0; j<N; ++j) {
                a(i, j) *= inv_pivot;
                inv(i, j) *= inv_pivot;
            }

            for(std::size_t row = 0; row < N; ++row) {
                if(row == i) continue;

                const T factor = a(row, i);

                for(std::size_t col=0; col<N; ++col) {
                    a(row, col) -= factor * a(i, col);
                    inv(row, col) -= factor * inv(i, col);
                }
            }
        }

        return inv;
    }
}