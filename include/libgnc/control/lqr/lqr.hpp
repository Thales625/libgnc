#pragma once

#include "libgnc/math/vector.hpp"

#include <cstddef>
#include <cmath>

namespace gnc::control
{
    template<typename T, std::size_t NX, std::size_t NU>
    class LQR {
    public:
        using StateVector = Vec<T, NX>;
        using InputVector = Vec<T, NU>;
        using ControlMatrix = Matrix<T, NU, NX>;

        ControlMatrix K;
        StateVector target_state; // x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate
        InputVector u; // thrust, roll_torque, pitch_torque, yaw_torque

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

    float force_to_throttle(float force) {
        constexpr float kt = 1e-6f;
        constexpr float max_ang_vel = 20e3f * 0.10472f;
        constexpr float max_thrust = kt * max_ang_vel * max_ang_vel;

        if (force < 0.0f) force = 0.0f;
        else if (force > max_thrust) force = max_thrust;

        return std::sqrtf(force / max_thrust);
    }

    Vec4f motor_mix(float thrust, const float roll_torque, const float pitch_torque, const float yaw_torque) {
        constexpr float width = 0.2f;
        constexpr float b_kt_ratio = 2e-8f / 1e-6f;

        const float delta_roll = roll_torque / (4.0f * width);
        const float delta_pitch = pitch_torque / (4.0f * width);
        const float delta_yaw = yaw_torque / (4.0f * b_kt_ratio);

        Vec4f throttle;

        thrust *= 0.25f;

        throttle[0] = force_to_throttle(thrust - delta_pitch + delta_yaw + delta_roll); // FR
        throttle[1] = force_to_throttle(thrust - delta_pitch - delta_yaw - delta_roll); // FL
        throttle[2] = force_to_throttle(thrust + delta_pitch - delta_yaw + delta_roll); // BR
        throttle[3] = force_to_throttle(thrust + delta_pitch + delta_yaw - delta_roll); // BL

        return throttle;
    }
}