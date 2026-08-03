#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_EXPECTED_CASTER_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_EXPECTED_CASTER_H

#include "cartan/expected.h"

#include <nanobind/nanobind.h>

#include <string>

namespace nanobind::detail
{

template <typename T, typename E>
struct type_caster<cartan::expected<T, E>>
{
    using value_caster = make_caster<T>;

    using expected_type = cartan::expected<T, E>;

    // The Python-visible type is the value type. On success the object is a T,
    // and on failure an exception propagates in place of a value, so there is
    // never an `Expected` object to annotate -- and a stub naming one would name
    // a type that does not exist in Python.
    NB_TYPE_CASTER(expected_type, value_caster::Name)

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
            if constexpr (requires { value.error().detail; })
                msg = value.error().detail;
            else if constexpr (requires { message(value.error()); })
                msg = message(value.error());
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
