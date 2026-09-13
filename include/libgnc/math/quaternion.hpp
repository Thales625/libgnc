#pragma once

#include <numbers>
#include <array>
#include <cmath>
#include <cstddef>

#ifdef ESP_PLATFORM
#include <esp_log.h>
#else
#include <iostream>
#endif

#include "utils.hpp"
#include "vector.hpp"

namespace gnc
{
    template<typename T>
    class Quat {
        public:
            // ctor
            constexpr Quat(const T w, const T x, const T y, const T z) : _w(w), _x(x), _y(y), _z(z) {};

            constexpr Quat() {
                _data.fill(T{});
            }

            constexpr explicit Quat(const T& value) {
                _data.fill(value);
            }

            constexpr Quat(std::initializer_list<T> init) {
                std::copy(init.begin(), init.end(), _data.begin());
            }

            // accessors
            constexpr T& operator[](std::size_t index) {
                return _data[index];
            }
            constexpr const T& operator[](std::size_t index) const {
                return _data[index];
            }

            constexpr T* data() {
                return _data.data();
            }
            constexpr const T* data() const {
                return _data.data();
            }

            // methods
            constexpr std::size_t size() const {
                return 4;
            }

            constexpr void print() const {
                #ifdef ESP_PLATFORM
                for(std::size_t i=0; i<4; ++i) {
                    ESP_LOGI("Quat::print", "%.2f", _data[i]);
                }
                #else
                for(std::size_t i=0; i<4; ++i) {
                    std::cout << _data[i] << std::endl;;
                }
                std::cout << std::endl;
                #endif
            }

            // quat norm
            constexpr T norm() const {
                return std::sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
            }

            // quat normalized
            constexpr Quat<T> normalized() const {
                T norm = this->norm();
                if (norm == T{}) {
                    return *this;
                }
                return *this / norm;
            }

            // quat normalize
            constexpr void normalize() {
                T norm = this->norm();
                if (norm == T{}) {
                    return;
                }
                *this /= norm;
            }

            // quat conjugate
            constexpr Quat<T> conjugate() const {
                return Quat<T> {-_w, _x, _y, _z};
            }

            // vector rotate
            constexpr Vec<T, 3> rotate_vector(const Vec<T, 3> vec) const {
                return Quat::rotate_vector(vec, *this);
            }

            constexpr std::array<T, 3> to_euler() const {
                T w = _w, x = _x, y = _y, z = _z;

                // roll (x-axis)
                T sinr_cosr = 2.0 * (w * x + y * z);
                T cosr_cosr = 1.0 - 2.0 * (x * x + y * y);
                T roll = std::atan2(sinr_cosr, cosr_cosr);

                // pitch (y-axis)
                T sinp = 2.0 * (w * y - z * x);
                T pitch = (std::abs(sinp) >= 1.0) ? gnc::utils::sign(sinp) * (gnc::utils::PI<T> / 2.0) : std::asin(sinp);

                // yaw (z-axis)
                T siny_cosy = 2.0 * (w * z + x * y);
                T cosy_cosy = 1.0 - 2.0 * (y * y + z * z);
                T yaw = std::atan2(siny_cosy, cosy_cosy);

                return std::array<T, 3>{roll, pitch, yaw};
            }

            // OPERATORS

            // quat addition in-place
            constexpr Quat& operator+=(const Quat& other) {
                _w += other._w;
                _x += other._x;
                _y += other._y;
                _z += other._z;
                return *this;
            }

            // quat subtraction in-place
            constexpr Quat& operator-=(const Quat& other) {
                _w -= other._w;
                _x -= other._x;
                _y -= other._y;
                _z -= other._z;
                return *this;
            }

            // scalar multiplication in-place
            constexpr Quat& operator*=(const T& scalar) {
                _w *= scalar;
                _x *= scalar;
                _y *= scalar;
                _z *= scalar;
                return *this;
            }

            // quat multiplication in-place
            constexpr Quat& operator*=(const Quat& other) {
                T new_w = _w*other._w - _x*other._x - _y*other._y - _z*other._z;
                T new_x = _w*other._x + _x*other._w + _y*other._z - _z*other._y;
                T new_y = _w*other._y - _x*other._z + _y*other._w + _z*other._x;
                T new_z = _w*other._z + _x*other._y - _y*other._x + _z*other._w;

                _w = new_w;
                _x = new_x;
                _y = new_y;
                _z = new_z;
                return *this;
            }

            // scalar division in-place
            constexpr Quat& operator/=(const T& scalar) {
                _w /= scalar;
                _x /= scalar;
                _y /= scalar;
                _z /= scalar;
                return *this;
            }

            // static methods
            static constexpr Quat<T> from_euler(T roll, T pitch, T yaw) {
                // @param roll roll angle in radians
                // @param pitch pitch angle in radians
                // @param yaw yaw angle in radians

                T cr = std::cos(roll / 2.0);
                T sr = std::sin(roll / 2.0);
                T cp = std::cos(pitch / 2.0);
                T sp = std::sin(pitch / 2.0);
                T cy = std::cos(yaw / 2.0);
                T sy = std::sin(yaw / 2.0);
                return Quat<T>(cr * cp * cy + sr * sp * sy,
                               sr * cp * cy - cr * sp * sy,
                               cr * sp * cy + sr * cp * sy,
                               cr * cp * sy - sr * sp * cy);
            }

            static constexpr Vec<T, 3> rotate_vector(const Vec<T, 3> vec, const Quat<T> quat) {
                T norm = quat.norm();
                if (norm < 1e-8) return vec;

                Quat<T> q_norm = quat / norm;

                // v' = q_unit * v_quat * q_conj
                Quat<T> result = (q_norm * (Quat<T>{0, vec[0], vec[1], vec[2]}) * q_norm.conjugate());

                return Vec<T, 3>(result._x, result._y, result._z);
            }

        private:
            union {
                std::array<T, 4> _data;
                struct {
                    T _w, _x, _y, _z;
                };
            };
    };

    // OPERATORS

    // quat addition
    template<typename T>
    constexpr Quat<T> operator+(Quat<T> lhs, const Quat<T>& rhs) {
        lhs += rhs;
        return lhs;
    }

    // quat subtraction
    template<typename T>
    constexpr Quat<T> operator-(Quat<T> lhs, const Quat<T>& rhs) {
        lhs -= rhs;
        return lhs;
    }

    // scalar multiplication
    template<typename T>
    constexpr Quat<T> operator*(Quat<T> quat, const T& scalar) {
        quat *= scalar;
        return quat;
    }
    template<typename T>
    constexpr Quat<T> operator*(const T& scalar, Quat<T> quat) {
        quat *= scalar;
        return quat;
    }

    // quat multiplication
    template<typename T>
    constexpr Quat<T> operator*(Quat<T> quat, const Quat<T>& other) {
        quat *= other;
        return quat;
    }

    // scalar division
    template<typename T>
    constexpr Quat<T> operator/(Quat<T> quat, const T& scalar) {
        quat /= scalar;
        return quat;
    }
}