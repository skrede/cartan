#include <liepp/types.h>

#include <iostream>

int main()
{
    liepp::vector3<double> v = liepp::vector3<double>::UnitX();
    std::cout << "liepp integration test PASSED (v.norm() = " << v.norm() << ")" << std::endl;
    return 0;
}
