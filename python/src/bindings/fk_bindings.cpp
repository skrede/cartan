#include "cartan/lie/se3.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "registrations.h"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace
{

using SE3d = cartan::se3<double>;
using KinematicChaind = cartan::kinematic_chain<double, cartan::dynamic>;
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

namespace cartan::python
{

void register_fk(nb::module_& m)
{
    m.def("forward_kinematics", &fk_end_effector,
          "Forward kinematics: chain + joint vector -> end-effector SE(3) pose.",
          nb::arg("chain"), nb::arg("q"));

    m.def("space_jacobian", &space_jacobian_q,
          "Space-frame Jacobian J_s(q) such that V_s = J_s(q) * dq.",
          nb::arg("chain"), nb::arg("q"));

    m.def("body_jacobian", &body_jacobian_q,
          "Body-frame Jacobian J_b(q) such that V_b = J_b(q) * dq.",
          nb::arg("chain"), nb::arg("q"));
}

}
