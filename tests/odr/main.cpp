#include "holder.h"

#include <cstdio>

int main()
{
    const std::size_t native = holder_size_native();
    const std::size_t other = holder_size_backend();
    const bool agree = native == other;

    std::printf("native=%zu other=%zu agree=%s\n", native, other, agree ? "yes" : "no");

    // Writing through the other unit's view of the layout is defined only once
    // the two agree; on a disagreement it is the stack overrun this gate forbids.
    if (!agree)
    {
        return 1;
    }

    holder h{};
    touch(h);
    return 0;
}
