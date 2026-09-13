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

#include "matrix.hpp"

namespace gnc
{
    template<typename T, std::size_t N>
    class Vec {
        public:
            // ctor
            constexpr Vec() {
                _data.fill(T{});
            }

            constexpr explicit Vec(const T& value) {
                _data.fill(value);
            }

            constexpr Vec(std::initializer_list<T> init) {
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

            template<std::size_t M>
            constexpr Vec<T, M> head() const {
                static_assert(M <= N);

                Vec<T, M> result;

                for(std::size_t i=0; i<M; ++i)
                    result[i] = _data[i];

                return result;
            }

            template<std::size_t M>
            constexpr Vec<T, M> tail() const {
                static_assert(M <= N);

                Vec<T, M> result;

                constexpr std::size_t start = N - M;

                for(std::size_t i=0; i<M; ++i)
                    result[i] = _data[start + i];

                return result;
            }

            template<std::size_t Start, std::size_t Length>
            constexpr Vec<T, Length> segment() const {
                static_assert(Start + Length <= N);

                Vec<T, Length> result;

                for(std::size_t i=0; i<Length; ++i)
                    result[i] = _data[Start + i];

                return result;
            }

            // methods
            constexpr std::size_t size() const {
                return N;
            }

            constexpr void print() const {
                #ifdef ESP_PLATFORM
                for(std::size_t i=0; i<N; ++i) {
                    ESP_LOGI("VEC", "%.2f", _data[i]);
                }
                #else
                for(std::size_t i=0; i<N; ++i) {
                    std::cout << _data[i] << std::endl;;
                }
                std::cout << std::endl;
                #endif
            }

            // vector transpose
            constexpr Matrix<T, 1, N> transpose() const {
                Matrix<T, 1, N> result;

                for(std::size_t i = 0; i < N; ++i)
                    result(0, i) = _data[i];

                return result;
            }

            // OPERATORS

            // vector assignment
            template<typename U>
            constexpr Vec& operator=(const U (&other)[N]) {
                for(std::size_t i=0; i<N; ++i)
                    _data[i] = static_cast<T>(other[i]);

                return *this;
            }

            // vector addition in-place
            constexpr Vec& operator+=(const Vec& other) {
                for(std::size_t i=0; i<N; ++i) {
                    _data[i] += other[i];
                }

                return *this;
            }

            // vector subtraction in-place
            constexpr Vec& operator-=(const Vec& other) {
                for(std::size_t i=0; i<N; ++i) {
                    _data[i] -= other[i];
                }

                return *this;
            }

            // scalar multiplication in-place
            constexpr Vec& operator*=(const T& scalar) {
                for(std::size_t i=0; i<N; ++i) {
                    _data[i] *= scalar;
                }

                return *this;
            }

            // scalar division in-place
            constexpr Vec& operator/=(const T& scalar) {
                for(std::size_t i=0; i<N; ++i) {
                    _data[i] /= scalar;
                }

                return *this;
            }

        private:
            std::array<T, N> _data;
    };

    // OPERATORS

    // vector addition
    template<typename T, std::size_t N>
    constexpr Vec<T, N> operator+(Vec<T, N> lhs, const Vec<T, N>& rhs) {
        lhs += rhs;
        return lhs;
    }

    // vector subtraction
    template<typename T, std::size_t N>
    constexpr Vec<T, N> operator-(Vec<T, N> lhs, const Vec<T, N>& rhs) {
        lhs -= rhs;
        return lhs;
    }

    // scalar multiplication
    template<typename T, std::size_t N>
    constexpr Vec<T, N> operator*(Vec<T, N> vec, const T& scalar) {
        vec *= scalar;
        return vec;
    }
    template<typename T, std::size_t N>
    constexpr Vec<T, N> operator*(const T& scalar, Vec<T, N> vec) {
        vec *= scalar;
        return vec;
    }

    // scalar division
    template<typename T, std::size_t N>
    constexpr Vec<T, N> operator/(Vec<T, N> vec, const T& scalar) {
        vec /= scalar;
        return vec;
    }

    using Vec2f = Vec<float, 2>;
    using Vec3f = Vec<float, 3>;
    using Vec4f = Vec<float, 4>;
}

// include specific operator implementations
#include "detail/vector_ops_generic.hpp"

#if defined(LIBGNC_USE_SIMD) && (LIBGNC_USE_SIMD == 1)
#include "detail/vector_ops_simd.hpp"
#endif

namespace gnc
{
    template<typename T, std::size_t N>
    constexpr T dot(const Vec<T, N>& a, const Vec<T, N>& b) {
        return detail::dot_impl(a, b);
    }

    template<typename T>
    constexpr Vec<T, 3> cross(const Vec<T, 3>& a, const Vec<T, 3>& b) {
        return detail::cross_impl(a, b);
    }

    template<typename T, std::size_t N>
    constexpr T squared_norm(const Vec<T, N>& v) {
        return detail::squared_norm_impl(v);
    }

    template<typename T, std::size_t N>
    T norm(const Vec<T, N>& v) {
        return detail::norm_impl(v);
    }

    template<typename T, std::size_t N>
    Vec<T, N> normalized(const Vec<T, N>& v) {
        const T n = norm(v);

        if(n == T{}) {
            return v;
        }

        return v / n;
    }

    template<typename T, std::size_t N>
    constexpr T sum(const Vec<T, N>& v) {
        T result{};

        for(std::size_t i=0; i<N; ++i) {
            result += v[i];
        }

        return result;
    }

    template<typename T, std::size_t N>
    constexpr T mean(const Vec<T, N>& v) {
        return sum(v) / static_cast<T>(N);
    }

    template<typename T, std::size_t N>
    constexpr T squared_distance(const Vec<T, N>& a, const Vec<T, N>& b) {
        return squared_norm(a - b);
    }

    template<typename T, std::size_t N>
    T distance(const Vec<T, N>& a, const Vec<T, N>& b) {
        return norm(a - b);
    }
}