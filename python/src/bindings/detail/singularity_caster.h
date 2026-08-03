#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_SINGULARITY_CASTER_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_SINGULARITY_CASTER_H

#include "cartan/expected.h"
#include "cartan/serial/fk/singularity_failure.h"

#include <nanobind/stl/optional.h>
#include <nanobind/nanobind.h>

#include <optional>

namespace nanobind::detail
{

/// Splits singularity_failure by kind rather than mapping the whole enum onto
/// one Python outcome, because the three values are not the same kind of event.
///
/// A configuration the chain cannot accept is a bad argument, so it raises
/// ValueError -- the exception forward_kinematics and both Jacobians already
/// raise for the same underlying chain_failure. An empty or entirely zero
/// spectrum is a well-formed question whose measure is simply undefined there,
/// so it answers None; raising would make a caller who did nothing wrong wrap a
/// try/except around a jointless chain or a zero Jacobian, both of which are
/// valid. The Python idiom is
/// `if (k := condition_number(chain, q)) is not None:`.
template <typename T>
struct type_caster<cartan::expected<T, cartan::singularity_failure>>
{
    using value_caster = make_caster<std::optional<T>>;

    using measured_type = cartan::expected<T, cartan::singularity_failure>;

    NB_TYPE_CASTER(measured_type, value_caster::Name)

    bool from_python(handle, uint8_t, cleanup_list*) noexcept
    {
        return false;
    }

    template <typename Measured>
    static handle from_cpp(Measured&& measured, rv_policy policy, cleanup_list* cleanup) noexcept
    {
        if (!measured.has_value())
        {
            if (measured.error() != cartan::singularity_failure::invalid_configuration)
                return none().release();

            PyErr_SetString(PyExc_ValueError, cartan::message(measured.error()));
            return handle();
        }
        return value_caster::from_cpp(
            std::optional<T>(std::forward<Measured>(measured).operator*()), policy, cleanup);
    }
};

}

#endif
