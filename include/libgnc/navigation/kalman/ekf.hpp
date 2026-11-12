#pragma once

#include "libgnc/math/matrix.hpp"
#include "libgnc/math/vector.hpp"
#include "libgnc/math/quaternion.hpp"

#include <cstddef>
#include <cmath>
#include <limits>

namespace gnc::navigation
{
    template<typename T>
    class AttitudeEKF {
    public:
        // State:
        // x = [qw, qx, qy, qz, bgx, bgy, bgz]
        static constexpr std::size_t NX = 7;
        static constexpr std::size_t NV = 3;

        using StateVector = Vec<T, NX>;
        using StateMatrix = Matrix<T, NX, NX>;

        using Vector3 = Vec<T, 3>;
        using Matrix3 = Matrix<T, 3, 3>;

        using Quaternion = Quat<T>;

    private:
        StateVector x_{};
        StateMatrix P_{StateMatrix::Identity()};
        StateMatrix Q_{StateMatrix::Zero()};

        // Numerical Jacobian perturbation.
        T epsilon_ = static_cast<T>(1e-6);

    private:
        // ---------------------------------------------------------
        // State / quaternion helpers
        // Quaternion ordering: [w, x, y, z]
        // ---------------------------------------------------------
        static Quaternion quaternion(const StateVector& x) {
            return Quaternion{x[0], x[1], x[2], x[3]};
        }

        static void set_quaternion(StateVector& x, const Quaternion& q) {
            x[0] = q[0];
            x[1] = q[1];
            x[2] = q[2];
            x[3] = q[3];
        }

        static void normalize_quaternion(StateVector& x) {
            auto q = quaternion(x);
            q = q.normalized();
            set_quaternion(x, q);
        }

        static T vector_norm(const Vector3& v) {
            return static_cast<T>(std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
        }

        static Vector3 normalize_vector(const Vector3& v) {
            const T n = vector_norm(v);

            if (!(n > std::numeric_limits<T>::epsilon()))
                return Vector3::zero();

            return v / n;
        }

        static void symmetrize(StateMatrix& P) {
            P = static_cast<T>(0.5) * (P + transpose(P));
        }

        // ---------------------------------------------------------
        // State transition
        //
        // Assumptions:
        // - q maps body coordinates to world coordinates.
        // - q_dot = 0.5 * q (*) [0, omega_body].
        // - Gyroscope measurements are in rad/s.
        // - Bias is in rad/s.
        // ---------------------------------------------------------
        StateVector transition(const StateVector& x, const Vector3& gyro, T dt) const {
            StateVector next = x;

            const T wx = gyro[0] - x[4];
            const T wy = gyro[1] - x[5];
            const T wz = gyro[2] - x[6];

            const T qw = x[0];
            const T qx = x[1];
            const T qy = x[2];
            const T qz = x[3];

            // q_dot = 0.5 * q (*) [0, wx, wy, wz]
            //
            // Written component-wise to avoid assuming a particular
            // quaternion multiplication API.
            next[0] = qw + static_cast<T>(0.5) * (-qx * wx - qy * wy - qz * wz) * dt;
            next[1] = qx + static_cast<T>(0.5) * ( qw * wx + qy * wz - qz * wy) * dt;
            next[2] = qy + static_cast<T>(0.5) * ( qw * wy + qz * wx - qx * wz) * dt;
            next[3] = qz + static_cast<T>(0.5) * ( qw * wz + qx * wy - qy * wx) * dt;

            // Random-walk bias model:
            // b(k+1) = b(k) + w_b
            next[4] = x[4];
            next[5] = x[5];
            next[6] = x[6];

            normalize_quaternion(next);

            return next;
        }

        // ---------------------------------------------------------
        // Predicted vector measurement
        //
        // reference_world: known reference vector in world coordinates.
        //
        // With q mapping body -> world:
        // reference_body = q_conjugate (*) reference_world (*) q
        //
        // This is suitable for gravity and magnetic-field direction
        // measurements, provided their reference conventions match.
        // ---------------------------------------------------------
        static Vector3 predict_vector_measurement(const StateVector& x, const Vector3& reference_world) {
            const auto q = quaternion(x);
            const auto q_inv = q.conjugate();

            const Quaternion reference_q{
                T{},
                reference_world[0],
                reference_world[1],
                reference_world[2]
            };

            const auto predicted_q = q_inv * reference_q * q;

            return normalize_vector(Vector3{
                predicted_q[1],
                predicted_q[2],
                predicted_q[3]
            });
        }

        // ---------------------------------------------------------
        // Numerical Jacobian of the state transition
        // F = df/dx
        // ---------------------------------------------------------
        StateMatrix transition_jacobian(const Vector3& gyro, T dt) const {
            StateMatrix F = StateMatrix::Zero();

            for (std::size_t j = 0; j < NX; ++j) {
                StateVector xp = x_;
                StateVector xm = x_;

                xp[j] += epsilon_;
                xm[j] -= epsilon_;

                normalize_quaternion(xp);
                normalize_quaternion(xm);

                const StateVector fp = transition(xp, gyro, dt);
                const StateVector fm = transition(xm, gyro, dt);

                for (std::size_t i = 0; i < NX; ++i)
                    F(i, j) = (fp[i] - fm[i]) / (static_cast<T>(2) * epsilon_);
            }

            return F;
        }

        // ---------------------------------------------------------
        // Numerical Jacobian of the vector measurement model
        // H = dh/dx
        // ---------------------------------------------------------
        Matrix<T, 3, NX> measurement_jacobian(const Vector3& reference_world) const {
            Matrix<T, 3, NX> H = Matrix<T, 3, NX>::Zero();

            for (std::size_t j = 0; j < NX; ++j) {
                StateVector xp = x_;
                StateVector xm = x_;

                xp[j] += epsilon_;
                xm[j] -= epsilon_;

                normalize_quaternion(xp);
                normalize_quaternion(xm);

                const Vector3 hp = predict_vector_measurement(xp, reference_world);
                const Vector3 hm = predict_vector_measurement(xm, reference_world);

                for (std::size_t i = 0; i < 3; ++i)
                    H(i, j) = (hp[i] - hm[i]) / (static_cast<T>(2) * epsilon_);
            }

            return H;
        }

    public:
        constexpr AttitudeEKF() = default;

        // ---------------------------------------------------------
        // Configuration / accessors
        // ---------------------------------------------------------
        void initialize(const Quaternion& q0, const Vector3& gyro_bias0, const StateMatrix& P0) {
            x_[0] = q0[0];
            x_[1] = q0[1];
            x_[2] = q0[2];
            x_[3] = q0[3];

            x_[4] = gyro_bias0[0];
            x_[5] = gyro_bias0[1];
            x_[6] = gyro_bias0[2];

            normalize_quaternion(x_);

            P_ = P0;
            symmetrize(P_);
        }

        void set_process_noise(const StateMatrix& Q) {
            Q_ = Q;
        }

        void set_covariance(const StateMatrix& P) {
            P_ = P;
            symmetrize(P_);
        }

        void set_jacobian_epsilon(T epsilon) {
            if (epsilon > T{}) epsilon_ = epsilon;
        }

        const StateVector& state() const {
            return x_;
        }

        const StateMatrix& covariance() const {
            return P_;
        }

        Quaternion attitude() const {
            return quaternion(x_);
        }

        Vector3 gyro_bias() const {
            return Vector3{x_[4], x_[5], x_[6]};
        }

        // ---------------------------------------------------------
        // Prediction
        //
        // gyro: angular velocity in rad/s
        // dt: elapsed time in seconds
        //
        // Q_ is the DISCRETE process covariance for this prediction.
        // ---------------------------------------------------------
        void predict(const Vector3& gyro, T dt) {
            if (!(dt > T{})) return;

            const StateMatrix F = transition_jacobian(gyro, dt);

            x_ = transition(x_, gyro, dt);

            P_ = F * P_ * transpose(F) + Q_;

            symmetrize(P_);
        }

        // ---------------------------------------------------------
        // Vector measurement update
        //
        // z:
        //   Measured direction in body coordinates.
        //
        // reference_world:
        //   Expected direction in world coordinates.
        //
        // R:
        //   3x3 covariance of the normalized vector measurement.
        //
        // Use separately for accelerometer and magnetometer.
        // ---------------------------------------------------------

        void update_vector(const Vector3& z, const Vector3& reference_world, const Matrix3& R) {
            const T zn = vector_norm(z);
            const T rn = vector_norm(reference_world);

            if (!(zn > std::numeric_limits<T>::epsilon()) || !(rn > std::numeric_limits<T>::epsilon())) return;

            const Vector3 z_norm = normalize_vector(z);
            const Vector3 ref_norm = normalize_vector(reference_world);

            const Vector3 h = predict_vector_measurement(x_, ref_norm);

            const Vector3 innovation = z_norm - h;

            const Matrix<T, 3, NX> H = measurement_jacobian(ref_norm);

            const Matrix3 S = H * P_ * transpose(H) + R;

            // TODO: replace explicit inverse with a linear solve
            // if/when the LibGNC matrix API supports it.
            const Matrix<T, NX, 3> K = P_ * transpose(H) * inverse(S);

            // Additive EKF correction.
            x_ += K * innovation;

            // Restore unit quaternion constraint.
            normalize_quaternion(x_);

            // Joseph covariance update.
            const StateMatrix I_KH = StateMatrix::Identity() - K * H;

            P_ = I_KH * P_ * transpose(I_KH) + K * R * transpose(K);

            symmetrize(P_);
        }

        // sensor updates
        void update_accelerometer(const Vector3& accel, const Vector3& gravity_reference_world, const Matrix3& R_accel) {
            update_vector(accel, gravity_reference_world, R_accel);
        }

        void update_magnetometer(const Vector3& mag, const Vector3& magnetic_reference_world, const Matrix3& R_mag) {
            update_vector(mag, magnetic_reference_world, R_mag);
        }
    };
}