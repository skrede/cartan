#include <cartan/frames/rotation.h>

struct A {};
struct B {};
struct C {};
struct D {};

int main()
{
    auto r1 = cartan::rotation<A, B, double>::identity();
    auto r2 = cartan::rotation<C, D, double>::identity();
    auto r3 = r1 * r2;
    return 0;
}
