#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_REGISTRATIONS_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_REGISTRATIONS_H

// This header carries Python.h into every binding translation unit, and Python.h
// sets feature-test macros the standard headers must be configured with, so it
// has to precede them: https://docs.python.org/3/extending/extending.html
#include <nanobind/nanobind.h>

namespace cartan::python
{

void register_lie(nanobind::module_& m);
void register_chain(nanobind::module_& m);
void register_fk(nanobind::module_& m);
void register_ik(nanobind::module_& m);
void register_analytical(nanobind::module_& m);
void register_exhaustive(nanobind::module_& m);

#ifdef CARTAN_PY_HAS_URDF
void register_urdf(nanobind::module_& m);
#endif

}

#endif
