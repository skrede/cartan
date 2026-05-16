#include "cartan/version.h"
#include "cartan/lie/so3.h"
#include "cartan/lie/se3.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/fk/forward_kinematics.h"
#include "cartan/serial/fk/jacobian.h"

#ifdef CARTAN_PY_HAS_URDF
#include "cartan/urdf.h"
#endif

#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>

#ifdef CARTAN_PY_HAS_URDF
#include <nanobind/stl/filesystem.h>
#endif

#include <vector>
#include <utility>

namespace nb = nanobind;

namespace
{

using SO3d = cartan::so3<double>;
using SE3d = cartan::se3<double>;
using ScrewAxisd = cartan::screw_axis<double>;
using JointLimitsd = cartan::joint_limits<double>;
using KinematicChaind = cartan::kinematic_chain<double, cartan::dynamic>;

using Vector3d = cartan::vector3<double>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using JacobianMatrixd = cartan::jacobian_matrix<double, cartan::dynamic>;

SE3d fk_end_effector(const KinematicChaind& chain, const VectorXd& q)
{
    return cartan::forward_kinematics(chain, q).end_effector;
}

JacobianMatrixd space_jacobian_q(const KinematicChaind& chain, const VectorXd& q)
{
    auto fk = cartan::forward_kinematics(chain, q);
    return cartan::space_jacobian(chain, fk);
}

JacobianMatrixd body_jacobian_q(const KinematicChaind& chain, const VectorXd& q)
{
    auto fk = cartan::forward_kinematics(chain, q);
    return cartan::body_jacobian(chain, fk);
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

    m.def("space_jacobian", &space_jacobian_q,
          "Space-frame Jacobian J_s(q) such that V_s = J_s(q) * dq.",
          nb::arg("chain"), nb::arg("q"));

    m.def("body_jacobian", &body_jacobian_q,
          "Body-frame Jacobian J_b(q) such that V_b = J_b(q) * dq.",
          nb::arg("chain"), nb::arg("q"));

#ifdef CARTAN_PY_HAS_URDF
    using UrdfErrord = cartan::urdf_error;
    using UrdfLoadResultd = cartan::urdf_load_result<double>;
    using UrdfMetadatad = cartan::urdf_metadata<double>;

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
#endif
}
