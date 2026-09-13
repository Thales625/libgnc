#pragma once

#include "libgnc/math/matrix.hpp"
#include "libgnc/math/vector.hpp"
#include "libgnc/math/quaternion.hpp"

#include <cstddef>

namespace gnc::navigation
{
    template<typename T, std::size_t NX, std::size_t NU, std::size_t NZ>
    class KalmanFilter {
    public:
        using ModelStateVector  = Vec<T, NX-1>;
        using StateVector       = Vec<T, NX>;
        using InputVector       = Vec<T, NU>;
        using MeasurementVector = Vec<T, NZ>;

        using StateMatrix       = Matrix<T, NX, NX>;
        using InputMatrix       = Matrix<T, NX, NU>;
        using MeasurementMatrix = Matrix<T, NZ, NX>;

        using ProcessNoiseMatrix     = Matrix<T, NX, NX>;
        using MeasurementNoiseMatrix = Matrix<T, NZ, NZ>;
        using CovarianceMatrix       = Matrix<T, NX, NX>;

    private:
        StateVector x_;
        CovarianceMatrix P_{CovarianceMatrix::Identity()};

        ProcessNoiseMatrix Q_{ProcessNoiseMatrix::Zero()};
        MeasurementNoiseMatrix R_{MeasurementNoiseMatrix::Zero()};

    public:
        constexpr KalmanFilter() = default;

        // setters & getters
        constexpr void set_state(const StateVector& x) { x_ = x; }
        constexpr void set_covariance(const CovarianceMatrix& P) { P_ = P; }
        constexpr void set_process_noise(const ProcessNoiseMatrix& Q) { Q_ = Q; }
        constexpr void set_measurement_noise(const MeasurementNoiseMatrix& R) { R_ = R; }

        constexpr const StateVector& state() const { return x_; }
        constexpr const CovarianceMatrix& covariance() const { return P_; }

        constexpr const ModelStateVector model_state() const {
            ModelStateVector state;

            auto rot = Quat {x_[6], x_[7], x_[8], x_[9]}.to_euler();

            state[0] = x_[0];
            state[1] = x_[1];
            state[2] = x_[2];

            state[3] = x_[3];
            state[4] = x_[4];
            state[5] = x_[5];

            state[6] = rot[0];
            state[7] = rot[1];
            state[8] = rot[2];

            state[9]  = x_[10];
            state[10] = x_[11];
            state[11] = x_[12];

            return state;
        }

        // methods
        constexpr void predict(const StateVector& x_next, const StateMatrix& F) {
            x_ = x_next;
            P_ = F * P_ * transpose(F) + Q_;
            P_ = static_cast<T>(0.5) * (P_ + transpose(P_));
        }

        constexpr void update(const MeasurementVector& z, const MeasurementMatrix& H, const MeasurementNoiseMatrix& R_adaptive) {
            const MeasurementVector y = z - (H * x_);
            const Matrix<T, NZ, NZ> S = H * P_ * transpose(H) + R_adaptive;

            // kalman gain
            const Matrix<T, NX, NZ> K = P_ * transpose(H) * inverse(S);

            // update state
            x_ += K * y;

            // Joseph stability form update
            const auto I_KH = CovarianceMatrix::Identity() - (K * H);

            P_ = I_KH * P_ * transpose(I_KH) + K * R_adaptive * transpose(K);
        }

        constexpr void update(const MeasurementVector& z, const MeasurementMatrix& H) {
            update(z, H, R_);
        }
    };
}