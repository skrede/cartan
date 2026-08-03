#include "cartan/lie/se3.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"
#include "cartan/serial/fk/singularity_analysis.h"
#include "cartan/serial/fk/singularity_failure.h"

#include "detail/expected_caster.h"
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

template <typename T>
using Measured = cartan::expected<T, cartan::singularity_failure>;

}

namespace cartan::python
{

void register_fk(nb::module_& m)
{
    m.def("forward_kinematics",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q) -> SE3d {
              auto fk = cartan::forward_kinematics(chain, q);
              if (!fk) throw nb::value_error(cartan::message(fk.error()));
              return fk->end_effector;
          },
          "Forward kinematics: chain + joint vector -> end-effector SE(3) pose.",
          nb::arg("chain"), nb::arg("q").noconvert());

    // Both Jacobians compute their own result from the chain they are given, so
    // the unwrap below establishes the shape precondition the checked Jacobian
    // would re-test; Python never supplies a result of its own.
    m.def("space_jacobian",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q) -> JacobianMatrixd {
              auto fk = cartan::forward_kinematics(chain, q);
              if (!fk) throw nb::value_error(cartan::message(fk.error()));
              return cartan::space_jacobian_unchecked(chain, *fk);
          },
          "Space-frame Jacobian J_s(q) such that V_s = J_s(q) * dq.",
          nb::arg("chain"), nb::arg("q").noconvert());

    m.def("body_jacobian",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q) -> JacobianMatrixd {
              auto fk = cartan::forward_kinematics(chain, q);
              if (!fk) throw nb::value_error(cartan::message(fk.error()));
              return cartan::body_jacobian_unchecked(chain, *fk);
          },
          "Body-frame Jacobian J_b(q) such that V_b = J_b(q) * dq.",
          nb::arg("chain"), nb::arg("q").noconvert());

    // Spectrum first, then the measures read off it: one decomposition answers
    // all four questions, and a per-measure (chain, q) form would hide four.
    // Each of them raises rather than returning None where it has no answer, so
    // the reason survives the crossing instead of collapsing into a falsy value.
    m.def("singular_values",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q,
             double length) -> Measured<VectorXd> {
              return cartan::singular_values(chain, q, length);
          },
          "Singular values of the body Jacobian at q, largest first, with the "
          "linear rows divided by length so they are commensurable with the "
          "angular ones. Raises for a chain with no joints, and for a q whose "
          "length disagrees with the chain or carries a non-finite component.",
          nb::arg("chain"), nb::arg("q").noconvert(), nb::arg("length") = 1.0);

    m.def("condition_number",
          [](const nb::DRef<const VectorXd>& sigma) -> Measured<double> {
              return cartan::condition_number(sigma);
          },
          "Ratio of largest to smallest singular value: one at an isotropic "
          "Jacobian, infinite at a singular one. Raises on an empty spectrum.",
          nb::arg("singular_values").noconvert());

    m.def("manipulability",
          [](const nb::DRef<const VectorXd>& sigma) -> Measured<double> {
              return cartan::manipulability(sigma);
          },
          "Yoshikawa's manipulability, the product of the singular values -- "
          "the manipulability ellipsoid's volume up to a constant factor. "
          "Raises on an empty spectrum, since the empty product of one would "
          "read as maximally manipulable.",
          nb::arg("singular_values").noconvert());

    m.def("isotropy",
          [](const nb::DRef<const VectorXd>& sigma) -> Measured<double> {
              return cartan::isotropy(sigma);
          },
          "Salisbury's isotropy index, the inverse condition number: one where "
          "the ellipsoid is a sphere, zero at a singularity. Raises on an empty "
          "spectrum, and on an entirely zero Jacobian, which has no ratio.",
          nb::arg("singular_values").noconvert());

    m.def("is_near_singular",
          [](const nb::DRef<const VectorXd>& sigma, double threshold) -> Measured<bool> {
              return cartan::is_near_singular(sigma, threshold);
          },
          "Whether the condition number is at or above threshold. Raises on an "
          "empty spectrum rather than answering a falsy None.",
          nb::arg("singular_values").noconvert(),
          nb::arg("threshold") = cartan::default_singularity_threshold_v<double>);

    m.def("is_near_singular",
          [](const KinematicChaind& chain,
             const nb::DRef<const VectorXd>& q,
             double threshold,
             double length) -> Measured<bool> {
              return cartan::is_near_singular(chain, q, threshold, length);
          },
          "is_near_singular at a configuration of the chain.",
          nb::arg("chain"), nb::arg("q").noconvert(),
          nb::arg("threshold") = cartan::default_singularity_threshold_v<double>,
          nb::arg("length") = 1.0);
}

}
