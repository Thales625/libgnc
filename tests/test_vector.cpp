#include <libgnc/math/vector.hpp>

int main() {
    gnc::Vec3f a{1,2,3};
    gnc::Vec3f b{4,5,6};

    auto d = gnc::cross(a,b);
    auto e = gnc::dot(a, b);

    d.print();
    std::cout << "Dot: " << e << std::endl;

    std::cout << "Transpose: " << std::endl;
    d.transpose().print();
}
