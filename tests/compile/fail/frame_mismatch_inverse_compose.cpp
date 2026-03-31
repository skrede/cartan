#include <liepp/frames/rotation.h>

struct A {};
struct B {};
struct C {};
struct D {};

int main()
{
    auto r1 = liepp::rotation<A, B, double>::identity();
    auto r2 = liepp::rotation<C, D, double>::identity();
    auto r3 = r1 * r2;
    return 0;
}
