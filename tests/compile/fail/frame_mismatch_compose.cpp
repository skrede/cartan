#include <cartan/frames/transform.h>

struct A {};
struct B {};
struct C {};
struct D {};

int main()
{
    auto t1 = cartan::transform<A, B, double>::identity();
    auto t2 = cartan::transform<C, D, double>::identity();
    auto t3 = t1 * t2;
    return 0;
}
