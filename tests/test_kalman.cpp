#include <iostream>

#include "libgnc/math/matrix.hpp"
#include "libgnc/math/vector.hpp"

#include "libgnc/navigation/kalman/ekf.hpp"

int main() {
    using T = float;
    using EKF = gnc::navigation::AttitudeEKF<T>;

    using StateMatrix = EKF::StateMatrix;
    using Vector3 = EKF::Vector3;
    using Matrix3 = EKF::Matrix3;
    using Quat = gnc::Quat<T>;

    Vector3 gravity_reference_world{
        0.0f,
        0.0f,
        1.0f
    };

    Vector3 magnetic_reference_world{
        0.8f,
        0.0f,
        0.6f
    };

    EKF ekf;

    float dt = 0.1f;

    StateMatrix P0 = StateMatrix::Identity();

    // Inicialização ilustrativa; a atitude real deve ser
    // determinada usando acelerômetro e magnetômetro.
    ekf.initialize(
        Quat{1.0f, 0.0f, 0.0f, 0.0f},
        Vector3{0.0f, 0.0f, 0.0f},
        P0
    );

    // Configurar Q e R a partir da caracterização dos sensores.
    StateMatrix Q = StateMatrix::Zero();
    ekf.set_process_noise(Q);

    Matrix3 R_accel = Matrix3::Identity();
    Matrix3 R_mag = Matrix3::Identity();

    // Loop de estimação:
    Vector3 gyro{1, 0, 0}; // rad/s
    Vector3 accel{0, 0, 1};
    Vector3 mag{0, 0, 0};

    ekf.predict(gyro, dt);

    // As referências devem estar expressas no referencial terrestre
    // e respeitar as convenções de sinal e eixos dos sensores.
    ekf.update_accelerometer(
        accel,
        gravity_reference_world,
        R_accel
    );

    ekf.update_magnetometer(
        mag,
        magnetic_reference_world,
        R_mag
    );

    const auto q = ekf.attitude();
    const auto bg = ekf.gyro_bias();

    std::cout << "Testing Kalman" << std::endl;

    q.print();


    return 0;
}