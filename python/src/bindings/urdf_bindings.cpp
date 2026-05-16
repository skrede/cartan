#include "registrations.h"

#ifdef CARTAN_PY_HAS_URDF

#include "cartan/urdf.h"

#include "detail/urdf_python_error.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/nanobind.h>

#include <utility>
#include <iterator>
#include <algorithm>

namespace nb = nanobind;

namespace
{

using UrdfLoadResultd = cartan::urdf_load_result<double>;
using UrdfMetadatad = cartan::urdf_metadata<double>;

}

namespace cartan::python
{

void register_urdf(nb::module_& m)
{
    nb::enum_<cartan::urdf::urdf_failure>(m, "UrdfFailure",
        "Failure mode categories returned by load_urdf.")
        .value("malformed_xml", cartan::urdf::urdf_failure::malformed_xml)
        .value("unsupported_joint_type", cartan::urdf::urdf_failure::unsupported_joint_type)
        .value("unknown_link_reference", cartan::urdf::urdf_failure::unknown_link_reference)
        .value("unknown_parent_link", cartan::urdf::urdf_failure::unknown_parent_link)
        .value("branched_kinematic_tree", cartan::urdf::urdf_failure::branched_kinematic_tree)
        .value("link_not_found", cartan::urdf::urdf_failure::link_not_found)
        .value("mimic_joint_unsupported", cartan::urdf::urdf_failure::mimic_joint_unsupported)
        .value("inertial_singular", cartan::urdf::urdf_failure::inertial_singular)
        .value("sdf_not_supported", cartan::urdf::urdf_failure::sdf_not_supported)
        .value("unknown_error", cartan::urdf::urdf_failure::unknown_error);

    // Create the Python exception class as a true subclass of RuntimeError via
    // the C API. nanobind's nb::class_ with PyExc_RuntimeError as base rejects
    // non-nanobind base types at runtime; nb::exception<T> only carries a
    // what() string and cannot expose kind/detail attributes. The C API path
    // creates a regular Python exception class, then we set `kind` and `detail`
    // on each instance via the translator.
    PyObject* urdf_error_cls = PyErr_NewExceptionWithDoc(
        "cartan._core.UrdfError",
        "URDF parse/extract failure. Carries `kind` (UrdfFailure enum) and "
        "`detail` (str).",
        PyExc_RuntimeError,
        /*dict=*/nullptr);
    if (!urdf_error_cls)
        throw nb::python_error();

    // Attach the class to the module under the name "UrdfError". The module
    // takes a new reference; we release ours via Py_DECREF after attaching.
    if (PyModule_AddObject(m.ptr(), "UrdfError", urdf_error_cls) < 0)
    {
        Py_DECREF(urdf_error_cls);
        throw nb::python_error();
    }
    // PyModule_AddObject steals the reference on success; reacquire one for
    // the translator-captured payload to keep the class alive for the module's
    // lifetime.
    Py_INCREF(urdf_error_cls);

    nb::register_exception_translator(
        [](const std::exception_ptr& p, void* payload) {
            auto* cls = static_cast<PyObject*>(payload);
            try
            {
                std::rethrow_exception(p);
            }
            catch (const cartan::detail::urdf_python_error& e)
            {
                nb::object py_kind = nb::cast(e.kind);
                nb::object py_detail = nb::cast(e.detail);
                PyObject* exc_obj = PyObject_CallFunctionObjArgs(
                    cls, py_detail.ptr(), nullptr);
                if (!exc_obj)
                    return;
                if (PyObject_SetAttrString(exc_obj, "kind", py_kind.ptr()) < 0)
                {
                    Py_DECREF(exc_obj);
                    return;
                }
                if (PyObject_SetAttrString(exc_obj, "detail", py_detail.ptr()) < 0)
                {
                    Py_DECREF(exc_obj);
                    return;
                }
                PyErr_SetObject(cls, exc_obj);
                Py_DECREF(exc_obj);
            }
        },
        urdf_error_cls);

    nb::class_<UrdfMetadatad>(m, "UrdfMetadata",
        "Strings and inertial properties accompanying a loaded chain.")
        .def_ro("base_link_name", &UrdfMetadatad::base_link_name)
        .def_ro("tool_link_name", &UrdfMetadatad::tool_link_name)
        .def_ro("joint_names", &UrdfMetadatad::joint_names)
        .def("joint_index",
             [](const UrdfMetadatad& meta, const std::string& name) -> int {
                 auto it = std::find(meta.joint_names.begin(),
                                     meta.joint_names.end(), name);
                 if (it == meta.joint_names.end())
                     throw nb::key_error(name.c_str());
                 return static_cast<int>(std::distance(meta.joint_names.begin(), it));
             },
             "Look up the joint index for a name. Raises KeyError if not found.",
             nb::arg("name"));

    nb::class_<UrdfLoadResultd>(m, "UrdfLoadResult",
        "Success value of load_urdf: kinematic chain + metadata side-table.")
        .def_ro("chain", &UrdfLoadResultd::chain)
        .def_ro("metadata", &UrdfLoadResultd::metadata);

    m.def("load_urdf",
          [](const std::filesystem::path& path) -> UrdfLoadResultd {
              auto result = cartan::load_urdf<double>(path);
              if (!result)
              {
                  auto err = std::move(result).error();
                  throw cartan::detail::urdf_python_error{err.kind, std::move(err.detail)};
              }
              return std::move(*result);
          },
          "Load a URDF document and return the extracted kinematic chain "
          "and metadata. Raises cartan.UrdfError on parse or extraction failure.",
          nb::arg("path"));
}

}

#endif
