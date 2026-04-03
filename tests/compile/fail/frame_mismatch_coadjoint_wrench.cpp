#include <cartan/frames/transform.h>
#include <cartan/frames/framed_wrench.h>

struct A {};
struct B {};
struct C {};

int main()
{
    auto T = cartan::transform<A, B, double>::identity();
    cartan::vector6<double> w;
    w.setZero();
    auto wr = cartan::framed_wrench<C, double>{w};
    auto result = cartan::coadjoint_map(T, wr);
    return 0;
}
