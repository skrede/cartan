#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_URDF_PYTHON_ERROR_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_URDF_PYTHON_ERROR_H

#include "cartan/urdf/error.h"

#include <string>
#include <utility>
#include <exception>

namespace cartan::detail
{

/// Binding-internal exception carrying the URDF failure kind and detail
/// string out of the load_urdf lambda. A nanobind exception translator
/// (registered by register_urdf) rewires this to the Python cartan.UrdfError
/// class.
struct urdf_python_error : std::exception
{
    cartan::urdf_failure kind;
    std::string detail;

    urdf_python_error(cartan::urdf_failure k, std::string d)
        : kind(k), detail(std::move(d)) {}

    [[nodiscard]] const char* what() const noexcept override
    {
        return detail.c_str();
    }
};

}

#endif
