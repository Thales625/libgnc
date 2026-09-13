#include "libgnc/math/matrix.hpp"
#include "libgnc/math/vector.hpp"

int main() {
    gnc::Matrix<float, 3, 3> A{1,2,3, 4,5,6, 7,8,9};
    gnc::Matrix<float, 3, 3> B{1,2,3, 4,5,6, 7,8,9};
    gnc::Matrix<float, 3, 3> P{1,2,3, 4,5,6, 7,8,9};
    gnc::Matrix<float, 3, 3> Q{1,2,3, 4,5,6, 7,8,9};

    gnc::Vec3f x{3, 2, 1};
    gnc::Vec3f u{3, 2, 1};

    auto x_next = A * x + B * u;
    auto P_next = A * P * transpose(A) + Q;

    x_next.print();
    P_next.print();

    std::cout << "DET A: " << gnc::determinant(A) << std::endl;

    return 0;

    gnc::Matrix<float, 3, 3> a{1,2,3, 4,5,6, 7,8,9};
    gnc::Matrix<float, 3, 3> b{1,2,3, 4,5,6, 7,8,9};

    gnc::Vec3f c{3, 2, 1};

    std::cout << "A:" << std::endl;
    a.print();
    std::cout << std::endl;

    // IDENTITY
    /*
    a.identity().print();
    std::cout << std::endl;
    gnc::Matrix<float, 3, 3>::Identity().print();
    return 0;
    */

    // ZERO
    /*
    a.zero().print();
    std::cout << std::endl;
    gnc::Matrix<float, 3, 3>::Zero().print();
    std::cout << std::endl;
    a.print();
    return 0;
    */

    std::cout << "B:" << std::endl;
    b.print();
    std::cout << std::endl;

    std::cout << "C:" << std::endl;
    c.print();
    std::cout << std::endl;

    auto d = a*c;

    std::cout << "D:" << std::endl;
    d.print();
    std::cout << std::endl;
}