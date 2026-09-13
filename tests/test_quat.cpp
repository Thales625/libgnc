#include <iostream>

#include <libgnc/math/quaternion.hpp>

int main() {
    auto quat = gnc::Quat<float> {1.0f, 0.15f, 0.0f, 0.0f};

    // auto quat = gnc::Quat<float>::fromEuler(0.15f, 0.0f, 0.0f);
    // std::cout << "NORM:" << quat.norm() << std::endl;

    quat.print();
    std::cout << "NORM:" << quat.norm() << std::endl;

    quat.normalize();

    quat.print();
    std::cout << "NORM:" << quat.norm() << std::endl;
}
