#include "registrations.h"

#ifdef CARTAN_PY_HAS_URDF

#include "cartan/urdf.h"

#include "detail/expected_caster.h"
#include "detail/urdf_python_error.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/nanobind.h>

#include <utility>

namespace nb = nanobind;

namespace
{

using UrdfErrord = cartan::urdf_error;
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

    nb::class_<UrdfErrord>(m, "UrdfError",
        "Diagnostic returned in the failure channel of load_urdf.")
        .def_ro("kind", &UrdfErrord::kind)
        .def_ro("detail", &UrdfErrord::detail);

    nb::class_<UrdfMetadatad>(m, "UrdfMetadata",
        "Strings and inertial properties accompanying a loaded chain.")
        .def_ro("base_link_name", &UrdfMetadatad::base_link_name)
        .def_ro("tool_link_name", &UrdfMetadatad::tool_link_name)
        .def_ro("joint_names", &UrdfMetadatad::joint_names);

    nb::class_<UrdfLoadResultd>(m, "UrdfLoadResult",
        "Success value of load_urdf: kinematic chain + metadata side-table.")
        .def_ro("chain", &UrdfLoadResultd::chain)
        .def_ro("metadata", &UrdfLoadResultd::metadata);

    m.def("load_urdf",
          [](const std::filesystem::path& path) -> UrdfLoadResultd {
              auto result = cartan::load_urdf<double>(path);
              if (!result) {
                  const auto& err = result.error();
                  throw std::runtime_error(
                      "cartan.load_urdf failed: " + err.detail);
              }
              return std::move(*result);
          },
          "Load a URDF document and return the extracted kinematic chain "
          "and metadata. Raises RuntimeError on parse or extraction failure.",
          nb::arg("path"));
}

}

#endif
