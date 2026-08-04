#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_URDF_PYTHON_ERROR_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_URDF_PYTHON_ERROR_H

#include "cartan/urdf/error.h"

#include <string>
#include <utility>
#include <exception>

namespace cartan::detail
{

/// Binding-internal exception carrying the URDF failure kind, detail string
/// and the description reader's own code out of the load_urdf lambda. A
/// nanobind exception translator (registered by register_urdf) rewires this to
/// the Python cartan.UrdfError class. meios_code is empty when the failure
/// arose after the description was read and so has no reader code to quote;
/// the translator turns that into None rather than an empty string.
struct urdf_python_error : std::exception
{
    cartan::urdf_failure kind;
    std::string detail;
    std::string meios_code;

    urdf_python_error(cartan::urdf_failure k, std::string d, std::string c)
        : kind(k), detail(std::move(d)), meios_code(std::move(c)) {}

    const char* what() const noexcept override
    {
        return detail.c_str();
    }
};

}

#endif
