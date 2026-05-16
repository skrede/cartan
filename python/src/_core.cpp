#include "cartan/version.h"

#include "bindings/registrations.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <string>

namespace nb = nanobind;

NB_MODULE(_core, m)
{
    m.doc() = "cartan C++20 Lie group / kinematics / IK library — Python bindings";
    m.attr("__version__") = std::string(cartan::version());

    cartan::python::register_lie(m);
    cartan::python::register_chain(m);
    cartan::python::register_fk(m);

#ifdef CARTAN_PY_HAS_URDF
    cartan::python::register_urdf(m);
#endif
}
