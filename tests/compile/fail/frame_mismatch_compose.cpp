#include <liepp/frames/transform.h>

struct A {};
struct B {};
struct C {};
struct D {};

int main()
{
    auto t1 = liepp::transform<A, B, double>::identity();
    auto t2 = liepp::transform<C, D, double>::identity();
    auto t3 = t1 * t2;
    return 0;
}
