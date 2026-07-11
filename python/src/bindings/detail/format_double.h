#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_FORMAT_DOUBLE_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_FORMAT_DOUBLE_H

/// Locale-free shortest-round-trip formatting of a double for __repr__ text.
/// snprintf/strtod route through the C locale's LC_NUMERIC (kept "C" by
/// CPython), sidestepping the C++ num_put / ctype facets that crash when a
/// static-libstdc++ wheel and numpy load two libstdc++ runtimes. The
/// std::to_chars floating-point overloads would be cleaner but are marked
/// unavailable below the macOS 13.3 SDK deployment target the wheels target.

#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace cartan::python
{

inline std::string format_double(double value)
{
    std::array<char, 32> buffer{};
    for (int precision = 1; precision < 17; ++precision)
    {
        const int written = std::snprintf(buffer.data(), buffer.size(), "%.*g", precision, value);
        if (written > 0 && std::strtod(buffer.data(), nullptr) == value)
        {
            return std::string(buffer.data(), static_cast<std::size_t>(written));
        }
    }
    const int written = std::snprintf(buffer.data(), buffer.size(), "%.17g", value);
    return std::string(buffer.data(), static_cast<std::size_t>(written > 0 ? written : 0));
}

}

#endif
