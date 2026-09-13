#include <iostream>

#include "libgnc/math/matrix.hpp"
#include "libgnc/math/vector.hpp"

#include "libgnc/navigation/kalman/kalman.hpp"

int main() {
    gnc::navigation::KalmanFilter<float, 3, 3, 3> kalman;

    std::cout << "Testing Kalman" << std::endl;

    return 0;
}