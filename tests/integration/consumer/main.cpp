#include <cartan/types.h>

#include <iostream>

int main()
{
    cartan::vector3<double> v = cartan::vector3<double>::UnitX();
    std::cout << "cartan integration test PASSED (v.norm() = " << v.norm() << ")" << std::endl;
    return 0;
}
