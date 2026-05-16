#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/lie/se3.h"

#include "registrations.h"
#include "detail/expected_caster.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>
#include <nanobind/nanobind.h>

#include <vector>
#include <optional>

namespace nb = nanobind;

namespace
{

using SE3d = cartan::se3<double>;
using Vector3d = cartan::vector3<double>;
using ScrewAxisd = cartan::screw_axis<double>;
using JointLimitsd = cartan::joint_limits<double>;
using KinematicChaind = cartan::kinematic_chain<double, cartan::dynamic>;

}

namespace cartan::python
{

void register_chain(nb::module_& m)
{
    nb::class_<ScrewAxisd>(m, "ScrewAxis", "Spatial screw axis (omega, v) in 6-vector form")
        .def_static("revolute", &ScrewAxisd::revolute,
                    "Revolute joint screw axis with rotation axis through point.",
                    nb::arg("axis"), nb::arg("point"))
        .def_static("prismatic", &ScrewAxisd::prismatic,
                    "Prismatic joint screw axis with translation direction.",
                    nb::arg("direction"))
        .def("omega",
             [](const ScrewAxisd& s) -> Vector3d { return s.omega(); },
             "Angular component.")
        .def("v",
             [](const ScrewAxisd& s) -> Vector3d { return s.v(); },
             "Linear component.")
        .def("to_vector", &ScrewAxisd::to_vector,
             "Export as 6-vector (omega, v).")
        .def("is_revolute", &ScrewAxisd::is_revolute)
        .def_static("from_vector",
            [](const nb::DRef<const cartan::vector6<double>>& v) -> ScrewAxisd {
                auto r = ScrewAxisd::from_vector(v);
                if (!r) throw nb::value_error(r.error().c_str());
                return *std::move(r);
            },
            "Construct a ScrewAxis from a 6-vector (omega, v). "
            "Raises ValueError on invalid axis (e.g., zero omega for revolute, "
            "or non-unit norm).",
            nb::arg("v").noconvert());

    nb::class_<JointLimitsd>(m, "JointLimits", "Joint position bounds and optional dynamic limits")
        .def("__init__",
             [](JointLimitsd* self, double position_min, double position_max,
                std::optional<double> velocity_max,
                std::optional<double> effort_max,
                std::optional<double> acceleration_max) {
                 new (self) JointLimitsd{position_min, position_max,
                                         velocity_max, effort_max, acceleration_max};
             },
             nb::arg("position_min"), nb::arg("position_max"),
             nb::kw_only(),
             nb::arg("velocity_max") = nb::none(),
             nb::arg("effort_max") = nb::none(),
             nb::arg("acceleration_max") = nb::none())
        .def_rw("position_min", &JointLimitsd::position_min)
        .def_rw("position_max", &JointLimitsd::position_max)
        .def_prop_ro("velocity_max",
            [](const JointLimitsd& s) { return s.velocity_max; },
            "Maximum joint velocity (Optional[float]).")
        .def_prop_ro("effort_max",
            [](const JointLimitsd& s) { return s.effort_max; },
            "Maximum joint effort / torque (Optional[float]).")
        .def_prop_ro("acceleration_max",
            [](const JointLimitsd& s) { return s.acceleration_max; },
            "Maximum joint acceleration (Optional[float]).")
        .def("contains", &JointLimitsd::contains, nb::arg("position"));

    nb::class_<KinematicChaind>(m, "KinematicChain", "PoE serial kinematic chain (dynamic N)")
        .def(nb::init<const SE3d&,
                      std::vector<ScrewAxisd>,
                      std::vector<JointLimitsd>>(),
             nb::arg("home"), nb::arg("axes"), nb::arg("limits"))
        .def("home", &KinematicChaind::home, nb::rv_policy::reference_internal)
        .def("num_joints", &KinematicChaind::num_joints)
        .def("axis", &KinematicChaind::axis,
             nb::arg("i"), nb::rv_policy::reference_internal)
        .def("axes", &KinematicChaind::axes, nb::rv_policy::reference_internal,
             "Space-frame screw axes (list view; lifetime tied to chain).")
        .def("limits", &KinematicChaind::limits, nb::rv_policy::reference_internal,
             "Joint limits (list view; lifetime tied to chain).");
}

}
