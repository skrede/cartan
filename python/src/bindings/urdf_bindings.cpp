#include "registrations.h"

#ifdef CARTAN_PY_HAS_URDF

#include "cartan/urdf.h"

#include "detail/urdf_python_error.h"

#include <meios/diagnostic/diagnostic_code.h>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/nanobind.h>

#include <string>
#include <utility>
#include <iterator>
#include <algorithm>

namespace nb = nanobind;

namespace
{

using UrdfLoadResultd = cartan::urdf_load_result<double>;
using UrdfMetadatad = cartan::urdf_metadata<double>;

/// The tier crosses as a string: it has three values against the reader code's
/// fifty-seven, and a fourth Python enum buys nothing a caller cannot compare
/// directly. No default arm, so a tier added later trips -Wswitch here.
const char* severity_name(cartan::urdf_severity severity)
{
    switch (severity)
    {
    case cartan::urdf_severity::error:
        return "error";
    case cartan::urdf_severity::warn:
        return "warn";
    case cartan::urdf_severity::info:
        break;
    }
    return "info";
}

std::string code_name(meios::diagnostic_code code)
{
    return std::string(meios::to_string(code));
}

}

namespace cartan::python
{

void register_urdf(nb::module_& m)
{
    nb::enum_<cartan::urdf_failure>(m, "UrdfFailure",
        "Failure mode categories returned by load_urdf.")
        .value("malformed_xml", cartan::urdf_failure::malformed_xml)
        .value("unsupported_joint_type", cartan::urdf_failure::unsupported_joint_type)
        .value("unknown_link_reference", cartan::urdf_failure::unknown_link_reference)
        .value("unknown_parent_link", cartan::urdf_failure::unknown_parent_link)
        .value("branched_kinematic_tree", cartan::urdf_failure::branched_kinematic_tree)
        .value("link_not_found", cartan::urdf_failure::link_not_found)
        .value("mimic_joint_unsupported", cartan::urdf_failure::mimic_joint_unsupported)
        .value("inertial_singular", cartan::urdf_failure::inertial_singular)
        .value("sdf_not_supported", cartan::urdf_failure::sdf_not_supported)
        .value("cyclic_kinematic_tree", cartan::urdf_failure::cyclic_kinematic_tree)
        .value("missing_joint_limit", cartan::urdf_failure::missing_joint_limit)
        .value("invalid_joint_limit", cartan::urdf_failure::invalid_joint_limit)
        .value("zero_axis", cartan::urdf_failure::zero_axis)
        .value("non_finite_value", cartan::urdf_failure::non_finite_value)
        .value("duplicate_name", cartan::urdf_failure::duplicate_name)
        .value("multi_parent_link", cartan::urdf_failure::multi_parent_link)
        .value("tool_link_unreachable", cartan::urdf_failure::tool_link_unreachable)
        .value("unknown_error", cartan::urdf_failure::unknown_error);

    // Create the Python exception class as a true subclass of RuntimeError via
    // the C API. nanobind's nb::class_ with PyExc_RuntimeError as base rejects
    // non-nanobind base types at runtime; nb::exception<T> only carries a
    // what() string and cannot expose kind/detail attributes. The C API path
    // creates a regular Python exception class, then we set `kind` and `detail`
    // on each instance via the translator.
    PyObject* urdf_error_cls = PyErr_NewExceptionWithDoc(
        "cartan._core.UrdfError",
        "URDF parse/extract failure. Carries `kind` (UrdfFailure enum), "
        "`detail` (str), and `meios_code` (str, or None when the failure arose "
        "after the description was read and has no reader code to quote).",
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
                nb::object py_code = e.meios_code.empty()
                    ? nb::none() : nb::cast(e.meios_code);
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
                if (PyObject_SetAttrString(exc_obj, "meios_code", py_code.ptr()) < 0)
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

    nb::class_<cartan::urdf_source_location>(m, "UrdfSourceLocation",
        "Where in the source description a diagnostic applies.")
        .def_ro("file", &cartan::urdf_source_location::file)
        .def_ro("line", &cartan::urdf_source_location::line)
        .def_ro("element", &cartan::urdf_source_location::element);

    nb::class_<cartan::urdf_diagnostic>(m, "UrdfDiagnostic",
        "One record the description reader reported, with the tier it "
        "reported it at: 'error', 'warn' or 'info'.")
        .def_prop_ro("severity",
                     [](const cartan::urdf_diagnostic& d) -> const char* {
                         return severity_name(d.severity);
                     })
        .def_prop_ro("meios_code",
                     [](const cartan::urdf_diagnostic& d) -> std::string {
                         return code_name(d.meios_code);
                     })
        .def_ro("location", &cartan::urdf_diagnostic::location)
        .def_ro("message", &cartan::urdf_diagnostic::message);

    nb::class_<UrdfLoadResultd>(m, "UrdfLoadResult",
        "Success value of load_urdf: kinematic chain, metadata side-table, "
        "and everything the description reader reported along the way.")
        .def_ro("chain", &UrdfLoadResultd::chain)
        .def_ro("metadata", &UrdfLoadResultd::metadata)
        .def_ro("diagnostics", &UrdfLoadResultd::diagnostics);

    m.def("load_urdf",
          [](const std::filesystem::path& path) -> UrdfLoadResultd {
              auto result = cartan::load_urdf<double>(path);
              if (!result)
              {
                  auto err = std::move(result).error();
                  std::string code =
                      err.meios_code ? code_name(*err.meios_code) : std::string();
                  throw cartan::detail::urdf_python_error{
                      err.kind, std::move(err.detail), std::move(code)};
              }
              return std::move(*result);
          },
          "Load a URDF or xacro document and return the extracted kinematic "
          "chain, metadata and diagnostics. Raises cartan.UrdfError on parse "
          "or extraction failure.",
          nb::arg("path"));
}

}

#endif
