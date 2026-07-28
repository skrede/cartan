#include <cartan/lie/so3.h>

#include <cstdio>

int main()
{
    const cartan::vector3<double> phi{0.1, 0.2, 0.3};
    const cartan::so3<double> rotation = cartan::so3<double>::exp(phi);
    const cartan::vector3<double> recovered = rotation.log();

    const double error = (phi - recovered).norm();
    std::printf("round-trip error: %g\n", error);
    return error < 1e-12 ? 0 : 1;
}
