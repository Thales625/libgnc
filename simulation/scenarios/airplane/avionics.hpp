#pragma once

#include "flight_software.hpp"

#include "libgnc/math/vector.hpp"
#include "libgnc/control/lqr/lqr.hpp"
#include "libgnc/navigation/kalman/kalman.hpp"

#include <cmath>
#include <iostream>

float force_to_throttle(float force) {
    constexpr float kt = 1e-6f;
    constexpr float max_ang_vel = 20e3f * 0.10472f;
    constexpr float max_thrust = kt * max_ang_vel * max_ang_vel;

    if (force < 0.0f) force = 0.0f;
    else if (force > max_thrust) force = max_thrust;

    return std::sqrtf(force / max_thrust);
}

#define NX 13
#define NU 4

class AirplaneAvionics : public FlightSoftware {
public:
    using StateVector = gnc::Vec<float, NX>;
    using ModelStateVector = gnc::Vec<float, NX-1>;

    // ctor
    constexpr AirplaneAvionics() = default;

    // navigation
    gnc::navigation::KalmanFilter<float, NX, NU, 4> kalman;

    // control
    gnc::control::LQR<float, NX-1, NU> lqr {
        {
            4.80832019e-14f, 6.22549384e-14f, 2.21940669e+00f, 6.22658596e-14f, -9.66585260e-14f, 9.93115279e-01f, 4.21581221e-13f, 2.29134942e-13f, -2.32441921e-13f, 5.36886797e-15f, 2.92592297e-15f, -5.50259603e-15f,
            2.35713257e-13f, -8.79382761e-01f, -1.00724061e-13f, 1.06956143e-13f, -6.58657716e-01f, 2.36744848e-13f, 1.98779029e+00f, 1.41074343e-13f, -7.00390166e-14f, 3.05219951e-01f, 1.49707101e-15f, -7.57390909e-16f,
            8.79382761e-01f, -2.59154527e-15f, 3.06298455e-13f, 6.58657716e-01f, 2.09995763e-15f, 1.29616246e-13f, -9.00466797e-15f, 1.98779029e+00f, -6.68462192e-14f, 1.27355413e-15f, 3.05219951e-01f, -4.55488606e-15f,
            -1.23479749e-13f, -1.65925864e-13f, -1.24719578e-13f, -9.92164260e-14f, -7.43011048e-14f, -1.29035408e-13f, -7.68045681e-14f, -2.12260236e-13f, 2.97591951e-01f, -4.42464180e-16f, -2.54424061e-15f, 3.05487208e-01f
        }, // LQR K
        {1.96133f, 0, 0, 0} // u_eq
    };

    // getters
    constexpr StateVector estimated_state() const {
        return this->kalman.state();
    }
    constexpr gnc::Vec3f estimated_position() const {
        return this->kalman.state().head<3>();
    }
    constexpr gnc::Vec3f estimated_velocity() const {
        return this->kalman.state().segment<3, 3>();
    }

    constexpr ModelStateVector get_target_state() const {
        return this->lqr.target_state;
    }

    // setters
    void set_state(const StateVector& state) {
        this->kalman.set_state(state);
    }
    void set_target_state(const ModelStateVector& target) {
        this->lqr.target_state = target;
    }

    // methods
    void setup() override {}

    void update() override {
        static uint32_t last_ut = 0;

        if (last_ut == 0) {
            last_ut = this->data.ut;
            return;
        }

        float dt = (this->data.ut - last_ut) * 0.001f;
        last_ut = this->data.ut;

        switch (this->data.phase) {
            case FlightPhase::PHASE_WAITING:
                // command throttle
                this->data.actuator_cmd[0] = 0.0f;
                this->data.actuator_cmd[1] = 0.0f;
                this->data.actuator_cmd[2] = 0.0f;
                this->data.actuator_cmd[3] = 0.0f;
                if (this->data.armed) {
                    this->data.phase = PHASE_FLIGHT;
                }
                break;

            case FlightPhase::PHASE_FLIGHT:
                if (!this->data.armed) {
                    this->data.phase = PHASE_WAITING;
                    break;
                }

                // control
                this->lqr.update(this->kalman.model_state());
                this->motor_mix();
                break;

            case FlightPhase::PHASE_AUTO_LAND:
                // command throttle
                this->data.actuator_cmd[0] = 0.0f;
                this->data.actuator_cmd[1] = 0.0f;
                this->data.actuator_cmd[2] = 0.0f;
                this->data.actuator_cmd[3] = 0.0f;
                break;

            default:
                break;
        }
    }

private:
    void motor_mix() {
        constexpr float width = 0.2f;
        constexpr float b_kt_ratio = 2e-8f / 1e-6f;

        const float thrust = 0.25 * this->lqr.u[0];
        const float roll_torque = this->lqr.u[1];
        const float pitch_torque = this->lqr.u[2];
        const float yaw_torque = this->lqr.u[3];

        const float delta_roll = roll_torque / (4.0f * width);
        const float delta_pitch = pitch_torque / (4.0f * width);
        const float delta_yaw = yaw_torque / (4.0f * b_kt_ratio);

        this->data.actuator_cmd[0] = force_to_throttle(thrust - delta_pitch + delta_yaw + delta_roll); // FR
        this->data.actuator_cmd[1] = force_to_throttle(thrust - delta_pitch - delta_yaw - delta_roll); // FL
        this->data.actuator_cmd[2] = force_to_throttle(thrust + delta_pitch - delta_yaw + delta_roll); // BR
        this->data.actuator_cmd[3] = force_to_throttle(thrust + delta_pitch + delta_yaw - delta_roll); // BL
    }
};

using Avionics = AirplaneAvionics;