#pragma once

#include "libgnc/math/vector.hpp"

#include <cstddef>

namespace gnc::control
{
    template<typename T, std::size_t NX, std::size_t NU>
    class LQR {
    public:
        using StateVector = Vec<T, NX>;
        using InputVector = Vec<T, NU>;
        using ControlMatrix = Matrix<T, NU, NX>;

        ControlMatrix K;
        StateVector target_state;
        InputVector u;

        InputVector u_eq;

        // ctor
        LQR() = default;
        LQR(const ControlMatrix K) : K(K) {}
        LQR(const ControlMatrix K, const InputVector u_eq) : K(K), u_eq(u_eq) {}

        // methods
        void update(const StateVector& current_state) {
            const StateVector error = this->target_state - current_state;

            this->u = this->K * error + this->u_eq;
        };
    };
}