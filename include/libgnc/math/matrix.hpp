#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>

#ifdef ESP_PLATFORM
#include <esp_log.h>
#else
#include <iostream>
#endif

#include "libgnc/config.hpp"
#include "detail/vector_fwd.hpp"

namespace gnc
{
    template <typename T, std::size_t Rows, std::size_t Cols>
    class Matrix
    {
        public:
            constexpr Matrix() {
                _data.fill(T{});
            }

            constexpr explicit Matrix(T value) {
                _data.fill(value);
            }

            constexpr Matrix(std::initializer_list<T> values) {
                auto it = values.begin();

                std::size_t i = 0;

                for (; it != values.end() && i < Rows * Cols; ++it, ++i)
                    _data[i] = *it;

                for (; i < Rows * Cols; ++i)
                    _data[i] = T{};
            }

            // accessors
            constexpr T& operator() (std::size_t row, std::size_t col) {
                return _data[row * Cols + col];
            }

            constexpr const T& operator() (std::size_t row, std::size_t col) const {
                return _data[row * Cols + col];
            }

            constexpr T* data() {
                return _data.data();
            }

            constexpr const T* data() const {
                return _data.data();
            }

            // methods
            constexpr void print() const {
                #ifdef ESP_PLATFORM
                for(std::size_t i=0; i<Rows; ++i) {
                    for(std::size_t j=0; j<Cols; ++j)
                        ESP_LOGI("Mat::print", "%.2f", _data[i*Cols + j]);
                    ESP_LOGI("Mat::print", "");
                }
                #else
                for(std::size_t i=0; i<Rows; ++i) {
                    for(std::size_t j=0; j<Cols; ++j)
                        std::cout << _data[i*Cols + j] << " ";
                    std::cout << std::endl;
                }
                #endif
            }

            // OPERATORS

            // matrix assignment
            template<typename U>
            constexpr Matrix& operator=(const U (&other)[Rows][Cols]) {
                for(std::size_t r=0; r<Rows; ++r)
                    for(std::size_t c=0; c<Cols; ++c)
                        _data[r*Cols + c] = static_cast<T>(other[r][c]);

                return *this;
            }
            constexpr Matrix& operator=(const Matrix& other) {
                for(std::size_t i=0; i<Rows*Cols; ++i)
                    _data[i] = other._data[i];

                return *this;
            }

            // matrix addition in-place
            constexpr Matrix& operator+=(const Matrix& other) {
                for(std::size_t i=0; i<Rows*Cols; ++i)
                    _data[i] += other._data[i];

                return *this;
            }

            // matrix subtraction in-place
            constexpr Matrix& operator-=(const Matrix& other) {
                for(std::size_t i=0; i<Rows*Cols; ++i)
                    _data[i] -= other._data[i];

                return *this;
            }

            // scalar multiplication in-place
            constexpr Matrix& operator*=(const T& scalar) {
                for(std::size_t i=0; i<Rows*Cols; ++i)
                    _data[i] *=  scalar;

                return *this;
            }

            // scalar division in-place
            constexpr Matrix& operator/=(const T& scalar) {
                for(std::size_t i=0; i<Rows*Cols; ++i)
                    _data[i] /= scalar;

                return *this;
            }

            // identity
            constexpr Matrix<T, Rows, Rows> identity() const {
                Matrix<T, Rows, Rows> result = {};

                for(std::size_t i=0; i<Rows; ++i)
                    result(i, i) = T(1);

                return result;
            }

            // zero
            constexpr Matrix zero() const {
                return Matrix{};
            }

            // static identity
            static constexpr Matrix Identity() {
                return Matrix{}.identity();
            }

            // static zero
            static constexpr Matrix Zero() {
                return Matrix{};
            }

        private:
            std::array<T, Rows * Cols> _data;
    };

    // OPERATORS

    // matrix addition
    template <typename T, std::size_t Rows, std::size_t Cols>
    constexpr Matrix<T, Rows, Cols> operator+(Matrix<T, Rows, Cols> lhs, const Matrix<T, Rows, Cols>& rhs) {
        lhs += rhs;
        return lhs;
    }

    // matrix subtraction
    template <typename T, std::size_t Rows, std::size_t Cols>
    constexpr Matrix<T, Rows, Cols> operator-(Matrix<T, Rows, Cols> lhs, const Matrix<T, Rows, Cols>& rhs) {
        lhs -= rhs;
        return lhs;
    }

    // scalar multiplication
    template <typename T, std::size_t Rows, std::size_t Cols>
    constexpr Matrix<T, Rows, Cols> operator*(Matrix<T, Rows, Cols> mat, const T& scalar) {
        mat *= scalar;
        return mat;
    }
    template <typename T, std::size_t Rows, std::size_t Cols>
    constexpr Matrix<T, Rows, Cols> operator*(const T& scalar, Matrix<T, Rows, Cols> mat) {
        mat *= scalar;
        return mat;
    }

    // scalar division
    template <typename T, std::size_t Rows, std::size_t Cols>
    constexpr Matrix<T, Rows, Cols> operator/(Matrix<T, Rows, Cols> mat, const T& scalar) {
        mat /= scalar;
        return mat;
    }

    using Mat2f = Matrix<float, 2, 2>;
    using Mat3f = Matrix<float, 3, 3>;
    using Mat4f = Matrix<float, 4, 4>;
}

// include specific operator implementations
#include "detail/matrix_ops_generic.hpp"

#if defined(LIBGNC_USE_SIMD) && (LIBGNC_USE_SIMD == 1)
#include "detail/matrix_ops_simd.hpp"
#endif

namespace gnc
{
    // matrix x matrix
    template<typename T, std::size_t Rows, std::size_t Cols, std::size_t OtherCols>
    constexpr auto operator*(const Matrix<T, Rows, Cols>& lhs, const Matrix<T, Cols, OtherCols>& rhs) {
        return detail::matmul_impl(lhs, rhs);
    }

    // matrix x vector
    template<typename T, std::size_t Rows, std::size_t Cols>
    constexpr auto operator*(const Matrix<T, Rows, Cols>& lhs, const Vec<T, Cols>& rhs) {
        return detail::matvec_impl(lhs, rhs);
    }

    // matrix transpose
    template<typename T, std::size_t Rows, std::size_t Cols>
    constexpr Matrix<T, Cols, Rows> transpose(const Matrix<T, Rows, Cols>& mat) {
        return detail::transpose_impl(mat);
    }

    // matrix trace
    template<typename T, std::size_t N>
    constexpr T trace(const Matrix<T, N, N>& mat) {
        return detail::trace_impl(mat);
    }

    // matrix determinant
    template<typename T, std::size_t N>
    constexpr T determinant(const Matrix<T, N, N>& mat) {
        return detail::determinant_impl(mat);
    }

    // matrix inverse
    template<typename T, std::size_t N>
    constexpr Matrix<T, N, N> inverse(const Matrix<T, N, N>& mat) {
        return detail::inverse_impl(mat);
    }
}