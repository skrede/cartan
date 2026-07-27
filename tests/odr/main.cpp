#include "holder.h"

#include <cstdio>

int main()
{
    const std::size_t native = holder_size_native();
    const std::size_t other  = holder_size_backend();
    const bool        agree  = native == other;

    std::printf("native=%zu other=%zu agree=%s\n", native, other, agree ? "yes" : "no");

    if (!agree)
    {
        return 1;
    }

    // Both units mangle touch() identically, so the linker resolves this call
    // across a layout disagreement it cannot see. Reaching it only once the two
    // sizes agree is what keeps the call itself well defined.
    holder h{};
    touch(h);
    return 0;
}
