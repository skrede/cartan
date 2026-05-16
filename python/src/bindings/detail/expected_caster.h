#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_EXPECTED_CASTER_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_EXPECTED_CASTER_H

#include "cartan/expected.h"

#include <nanobind/nanobind.h>

#include <string>
#include <exception>

namespace cartan::detail
{

/// Binding-internal exception thrown by the caster when an expected arrives
/// with !has_value() and the call site did not pre-unwrap. Per-site lambdas
/// typically pre-unwrap and throw ValueError or a typed exception, so this
/// fallback path is the "no policy chosen" branch and intentionally a generic
/// RuntimeError shape on the Python side.
struct expected_unwrap_error : std::exception
{
    std::string detail;

    explicit expected_unwrap_error(std::string msg) : detail(std::move(msg)) {}

    [[nodiscard]] const char* what() const noexcept override
    {
        return detail.c_str();
    }
};

}

namespace nanobind::detail
{

template <typename T, typename E>
struct type_caster<cartan::expected<T, E>>
{
    using value_caster = make_caster<T>;

    using expected_type = cartan::expected<T, E>;

    NB_TYPE_CASTER(expected_type,
                   const_name("Expected[") + value_caster::Name + const_name("]"))

    // Python -> C++ direction is unused (caster is C++ -> Python only by
    // design). Returning false signals to nanobind that no conversion is
    // possible from a Python value.
    bool from_python(handle, uint8_t, cleanup_list*) noexcept
    {
        return false;
    }

    template <typename T_>
    static handle from_cpp(T_&& value, rv_policy policy, cleanup_list* cleanup) noexcept
    {
        if (!value.has_value())
        {
            // Fallback: convert error to string and set a Python RuntimeError.
            // Per-site lambdas pre-unwrap to avoid this path and raise their
            // preferred exception type (ValueError, UrdfError, ...).
            std::string msg;
            if constexpr (std::is_convertible_v<E, std::string>)
                msg = value.error();
            else if constexpr (requires { value.error().detail; })
                msg = value.error().detail;
            else
                msg = "cartan::expected unwrap on the C++ -> Python boundary";

            PyErr_SetString(PyExc_RuntimeError, msg.c_str());
            return handle();
        }
        return value_caster::from_cpp(std::forward<T_>(value).operator*(),
                                      policy, cleanup);
    }
};

}

#endif
