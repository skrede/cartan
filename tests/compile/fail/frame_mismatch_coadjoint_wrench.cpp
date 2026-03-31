#include <liepp/frames/transform.h>
#include <liepp/frames/framed_wrench.h>

struct A {};
struct B {};
struct C {};

int main()
{
    auto T = liepp::transform<A, B, double>::identity();
    liepp::vector6<double> w;
    w.setZero();
    auto wr = liepp::framed_wrench<C, double>{w};
    auto result = liepp::coadjoint_map(T, wr);
    return 0;
}
