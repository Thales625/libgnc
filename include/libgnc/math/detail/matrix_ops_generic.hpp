#pragma once

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

    // identity
    template<typename T, std::size_t N>
    constexpr Matrix<T, N, N> identity_impl() {
        Matrix<T, N, N> result;

        for(std::size_t i=0; i<N; ++i)
            result(i, i) = T(1);

        return result;
    }
}