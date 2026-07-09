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

}

namespace cartan::python
{

void register_fk(nb::module_& m)
{
    m.def("forward_kinematics",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q) -> SE3d {
              return cartan::forward_kinematics(chain, VectorXd(q)).end_effector;
          },
          "Forward kinematics: chain + joint vector -> end-effector SE(3) pose.",
          nb::arg("chain"), nb::arg("q").noconvert());

    m.def("space_jacobian",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q) -> JacobianMatrixd {
              VectorXd q_owned(q);
              auto fk = cartan::forward_kinematics(chain, q_owned);
              return cartan::space_jacobian<double, cartan::dynamic>(chain, fk);
          },
          "Space-frame Jacobian J_s(q) such that V_s = J_s(q) * dq.",
          nb::arg("chain"), nb::arg("q").noconvert());

    m.def("body_jacobian",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q) -> JacobianMatrixd {
              VectorXd q_owned(q);
              auto fk = cartan::forward_kinematics(chain, q_owned);
              return cartan::body_jacobian<double, cartan::dynamic>(chain, fk);
          },
          "Body-frame Jacobian J_b(q) such that V_b = J_b(q) * dq.",
          nb::arg("chain"), nb::arg("q").noconvert());
}

}
