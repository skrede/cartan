#include "holder.h"

#include <cstring>

std::size_t holder_size_backend()
{
    return sizeof(holder);
}

void touch(holder& h)
{
    std::memset(static_cast<void*>(&h), 0, sizeof(holder));
}
