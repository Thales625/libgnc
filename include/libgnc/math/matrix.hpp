#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>

#ifdef ESP_PLATFORM
#include <esp_log.h>
#else
#include <iostream>
#endif

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
                for(std::size_t i = 0; i < Rows; ++i) {
                    for(std::size_t j = 0; j < Cols; ++j)
                        ESP_LOGI("Mat::print", "%.2f", _data[i * Cols + j]);
                    ESP_LOGI("Mat::print", "");
                }
                #else
                for(std::size_t i = 0; i < Rows; ++i) {
                    for(std::size_t j = 0; j < Cols; ++j)
                        std::cout << _data[i * Cols + j] << " ";
                    std::cout << std::endl;
                }
                #endif
            }

            // operators
            constexpr Matrix operator+(const Matrix& rhs) const {
                Matrix result;

                for(std::size_t i=0; i<Rows*Cols; ++i)
                    result._data[i] = _data[i] + rhs._data[i];

                return result;
            }

            constexpr Matrix& operator+=(const Matrix& rhs) {
                for(std::size_t i=0; i<Rows*Cols; ++i)
                    _data[i] += rhs._data[i];

                return *this;
            }

            // subtraction
            constexpr Matrix operator-(const Matrix& rhs) const {
                Matrix result;

                for(std::size_t i=0; i<Rows*Cols; ++i)
                    result._data[i] = _data[i] - rhs._data[i];

                return result;
            }

            constexpr Matrix& operator-=(const Matrix& rhs) {
                for(std::size_t i=0; i<Rows*Cols; ++i)
                    _data[i] -= rhs._data[i];

                return *this;
            }

            // scalar
            constexpr Matrix operator*(T scalar) const {
                Matrix result;

                for(std::size_t i=0; i<Rows*Cols; ++i)
                    result._data[i] = _data[i] * scalar;

                return result;
            }

            constexpr Matrix operator/(T scalar) const {
                Matrix result;

                for(std::size_t i=0; i<Rows*Cols; ++i)
                    result._data[i] = _data[i] / scalar;

                return result;
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
}

// include specific operator implementations
#include "detail/matrix_ops_generic.hpp"

#ifdef LIBGNC_USE_SIMD
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
}