#pragma once

#include "vector_fwd.hpp"
#include <cmath>

namespace gnc::detail
{
    template<typename T, std::size_t N>
    constexpr T dot_impl(const Vec<T, N>& a, const Vec<T, N>& b) {
        T result{};

        for (std::size_t i=0; i<N; ++i) {
            result += a[i] * b[i];
        }

        return result;
    }

    template<typename T>
    constexpr Vec<T, 3> cross_impl(const Vec<T, 3>& a, const Vec<T, 3>& b) {
        return Vec<T, 3>{
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    template<typename T, std::size_t N>
    constexpr T squared_norm_impl(const Vec<T, N>& v) {
        return dot_impl(v, v);
    }

    template<typename T, std::size_t N>
    T norm_impl(const Vec<T, N>& v) {
        return std::sqrt(squared_norm_impl(v));
    }
}