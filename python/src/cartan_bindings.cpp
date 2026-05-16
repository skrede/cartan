#include "cartan/version.h"
#include "cartan/lie/so3.h"
#include "cartan/lie/se3.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>

#include <vector>

namespace nb = nanobind;

namespace
{

using SO3d = cartan::so3<double>;
using SE3d = cartan::se3<double>;
using ScrewAxisd = cartan::screw_axis<double>;
using JointLimitsd = cartan::joint_limits<double>;
using KinematicChaind = cartan::kinematic_chain<double, cartan::dynamic>;

using Vector3d = cartan::vector3<double>;
using Vector6d = cartan::vector6<double>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

SE3d fk_end_effector(const KinematicChaind& chain, const VectorXd& q)
{
    return cartan::forward_kinematics(chain, q).end_effector;
}

}

NB_MODULE(_core, m)
{
    m.doc() = "cartan C++23 Lie group / kinematics / IK library — Python bindings";
    m.attr("__version__") = std::string(cartan::version());

    nb::class_<SO3d>(m, "SO3", "3D rotation group element (unit quaternion internal)")
        .def_static("identity", &SO3d::identity)
        .def_static("exp", &SO3d::exp,
                    "Exponential map: rotation vector (3,) -> SO(3).",
                    nb::arg("omega"))
        .def("log", &SO3d::log,
             "Logarithmic map: SO(3) -> rotation vector (3,).")
        .def("matrix", &SO3d::matrix,
             "Return the 3x3 rotation matrix.")
        .def("inverse", &SO3d::inverse)
        .def("act",
             [](const SO3d& self, const Vector3d& p) -> Vector3d {
                 return self.act(p);
             },
             "Rotate a 3D point.",
             nb::arg("p"))
        .def("__mul__",
             [](const SO3d& a, const SO3d& b) -> SO3d {
                 return a * b;
             });

    nb::class_<SE3d>(m, "SE3", "Rigid-body transformation group element")
        .def_static("identity", &SE3d::identity)
        .def_static("exp", &SE3d::exp,
                    "Exponential map: twist (6,) in (omega, rho) order -> SE(3).",
                    nb::arg("twist"))
        .def("log", &SE3d::log,
             "Logarithmic map: SE(3) -> twist (6,) in (omega, rho) order.")
        .def("matrix", &SE3d::matrix,
             "Return the 4x4 homogeneous transformation matrix.")
        .def("inverse", &SE3d::inverse)
        .def("adjoint", &SE3d::adjoint,
             "Return the 6x6 adjoint matrix (omega-first twist convention).")
        .def("act",
             [](const SE3d& self, const Vector3d& p) -> Vector3d {
                 return self.act(p);
             },
             "Transform a 3D point: R p + t.",
             nb::arg("p"))
        .def("__mul__",
             [](const SE3d& a, const SE3d& b) -> SE3d {
                 return a * b;
             });

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

    m.def("forward_kinematics", &fk_end_effector,
          "Forward kinematics: chain + joint vector -> end-effector SE(3) pose.",
          nb::arg("chain"), nb::arg("q"));
}
