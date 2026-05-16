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
        .def("is_revolute", &ScrewAxisd::is_revolute);

    nb::class_<JointLimitsd>(m, "JointLimits", "Joint position bounds and optional dynamic limits")
        .def("__init__",
             [](JointLimitsd* self, double pos_min, double pos_max) {
                 new (self) JointLimitsd{pos_min, pos_max, {}, {}, {}};
             },
             nb::arg("position_min"), nb::arg("position_max"))
        .def_rw("position_min", &JointLimitsd::position_min)
        .def_rw("position_max", &JointLimitsd::position_max)
        .def("contains", &JointLimitsd::contains, nb::arg("position"));

    nb::class_<KinematicChaind>(m, "KinematicChain", "PoE serial kinematic chain (dynamic N)")
        .def(nb::init<const SE3d&,
                      std::vector<ScrewAxisd>,
                      std::vector<JointLimitsd>>(),
             nb::arg("home"), nb::arg("axes"), nb::arg("limits"))
        .def("home", &KinematicChaind::home, nb::rv_policy::reference_internal)
        .def("num_joints", &KinematicChaind::num_joints)
        .def("axis", &KinematicChaind::axis,
             nb::arg("i"), nb::rv_policy::reference_internal);
}

}
