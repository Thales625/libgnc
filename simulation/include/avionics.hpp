#pragma once

#include "libgnc/navigation/kalman/kalman.hpp"
#include "libgnc/control/lqr/lqr.hpp"

#include "libgnc/math/vector.hpp"

#include <iostream>

#define NX 13
#define NU 4

enum FlightPhase {
    PHASE_WAITING,
    PHASE_FLIGHT,
    PHASE_AUTO_LAND
};

class Avionics {
public:
    using StateVector = gnc::Vec<float, NX>;
    using ModelStateVector = gnc::Vec<float, NX-1>;

    uint32_t ut;
    FlightPhase phase;
    gnc::Vec3f accel;
    gnc::Vec3f ang_vel;
    float altitude_baro;

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

    // command
    bool armed;
    gnc::Vec4f rotor_cmd;

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

    // setters
    void set_state(const StateVector& state) {
        this->kalman.set_state(state);
    }
    void set_target_state(const ModelStateVector& target) {
        this->lqr.target_state = target;
    }

    // ctor
    constexpr Avionics() = default;

    // methods
    void update() {
        static uint32_t last_ut = 0;

        if (last_ut == 0) {
            last_ut = this->ut;
            return;
        }

        float dt = (this->ut - last_ut) * 0.001f;
        last_ut = this->ut;

        /*
        // std::cout << "dt: " << dt << " | State: " << this->phase << std::endl;
        std::cout << "x: " << this->kalman.state()[0] << " ";
        std::cout << "y: " << this->kalman.state()[1] << " ";
        std::cout << "z: " << this->kalman.state()[2] << " ";
        std::cout << std::endl;
        */

        // navigation
        // kalman_predict(&core->state.kalman, dt, &core->state.accel, &core->state.ang_vel);
        // kalman_update_accel(&core->state.kalman, &core->state.accel);

        switch (this->phase) {
            case PHASE_WAITING:
                // command throttle
                this->rotor_cmd[0] = 0.0f;
                this->rotor_cmd[1] = 0.0f;
                this->rotor_cmd[2] = 0.0f;
                this->rotor_cmd[3] = 0.0f;
                if (this->armed) {
                    this->phase = PHASE_FLIGHT;
                }
                break;

            case PHASE_FLIGHT:
                if (!this->armed) {
                    this->phase = PHASE_WAITING;
                    break;
                }

                // control
                this->lqr.update(this->kalman.model_state());
                this->rotor_cmd = gnc::control::motor_mix(this->lqr.u[0], this->lqr.u[1], this->lqr.u[2], this->lqr.u[3]);

                /*
                // altitude
                std::cout << "altitude: " << this->altitude_baro << " ";
                std::cout << std::endl;
                */

                /*
                // accel
                std::cout << "accel_x: " << this->accel[0] << " ";
                std::cout << "accel_y: " << this->accel[1] << " ";
                std::cout << "accel_z: " << this->accel[2] << " ";
                std::cout << std::endl;
                */

                /*
                // ang_vel
                std::cout << "ang_vel_x: " << this->ang_vel[0] << " ";
                std::cout << "ang_vel_y: " << this->ang_vel[1] << " ";
                std::cout << "ang_vel_z: " << this->ang_vel[2] << " ";
                std::cout << std::endl;
                */

                /*
                // position
                std::cout << "x: " << model_state[0] << " ";
                std::cout << "y: " << model_state[1] << " ";
                std::cout << "z: " << model_state[2] << " ";
                std::cout << std::endl;
                */

                /*
                // control input
                std::cout << "thrust: " << this->lqr.u[0] << " ";
                std::cout << "pitch: " << this->lqr.u[1] << " ";
                std::cout << "roll: " << this->lqr.u[2] << " ";
                std::cout << "yaw: " << this->lqr.u[3] << " ";
                std::cout << std::endl;
                */
                break;

            case PHASE_AUTO_LAND:
                // command throttle
                this->rotor_cmd[0] = 0.0f;
                this->rotor_cmd[1] = 0.0f;
                this->rotor_cmd[2] = 0.0f;
                this->rotor_cmd[3] = 0.0f;
                break;

            default:
                break;
        }
    }
};