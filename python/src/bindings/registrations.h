#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_REGISTRATIONS_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_REGISTRATIONS_H

#include <nanobind/nanobind.h>

namespace cartan::python
{

void register_lie(nanobind::module_& m);
void register_chain(nanobind::module_& m);
void register_fk(nanobind::module_& m);

#ifdef CARTAN_PY_HAS_URDF
void register_urdf(nanobind::module_& m);
#endif

}

#endif
