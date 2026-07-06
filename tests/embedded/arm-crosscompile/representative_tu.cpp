/// Cartan bare-metal ARM cross-compile proof. Instantiates a representative
/// slice of the public surface — a small kinematic chain, forward kinematics,
/// the body Jacobian, a projected Levenberg-Marquardt IK step, and a closed-
/// form Paden-Kahan subproblem — using `float` scalars so the templates are
/// fully instantiated against the embedded-typical floating-point type.
///
/// Unlike the ESP-IDF smoke, this translation unit pulls in no vendor SDK: it
/// depends only on the header-only cartan libraries and Eigen. The whole point
/// is a bare header-only proof, so the work lives in a non-template `app_stub()`
/// that ODR-uses every instantiation and funnels the results into a single
/// value the optimizer cannot elide. Continuous integration compiles this with
/// `-c` (no link, no runtime), so no target startup code is required.

#include "cartan/analytical/paden_kahan.h"

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"

#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/serial/ik/solver/projected_lm.h"

#include <Eigen/Dense>

namespace
{

/// Build a planar 3R chain identical in spirit to the ESP-IDF smoke: three
/// revolute joints about +z with unit link spacing and a home pose one unit
/// past the last joint.
cartan::kinematic_chain<float, 3> build_planar_3r()
{
    using vec3f = cartan::vector3<float>;
    using screw = cartan::screw_axis<float>;

    auto s1 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(0.f, 0.f, 0.f));
    auto s2 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(1.f, 0.f, 0.f));
    auto s3 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(2.f, 0.f, 0.f));

    using rot = cartan::so3<float>;
    using pose = cartan::se3<float>;
    pose home(rot::identity(), vec3f(3.f, 0.f, 0.f));

    using limits = cartan::joint_limits<float>;
    return cartan::kinematic_chain<float, 3>{
        home,
        {s1, s2, s3},
        {limits{-3.14f, 3.14f}, limits{-3.14f, 3.14f}, limits{-3.14f, 3.14f}}};
}

/// Exercise forward kinematics, the body Jacobian, one projected-LM IK step,
/// and Paden-Kahan subproblem 1, folding every result into a single float so
/// none of the instantiations are dead-code-eliminated.
float app_stub()
{
    auto chain = build_planar_3r();

    Eigen::Matrix<float, 3, 1> q;
    q << 0.1f, 0.2f, 0.3f;

    // Forward kinematics: reach the end-effector pose at this configuration.
    auto fk = cartan::forward_kinematics(chain, q);
    const float fk_x = fk.end_effector.translation().x();

    // Body Jacobian: 6xN twist map at the same configuration.
    auto J_b = cartan::body_jacobian(chain, fk);
    const float jac_trace = J_b.cwiseAbs().sum();

    // Projected Levenberg-Marquardt: aim the solver at the just-computed
    // reachable pose from a nearby seed and advance a single work unit. This
    // ODR-uses the allocation-free active-set solve on a fixed-size chain.
    cartan::projected_lm<cartan::kinematic_chain<float, 3>> solver;
    cartan::convergence_criteria<float> criteria;

    Eigen::Matrix<float, 3, 1> q_seed;
    q_seed << 0.05f, 0.15f, 0.25f;

    solver.setup(chain, fk.end_effector, q_seed, criteria);
    auto ik_step = solver.step(chain, 1);
    const float ik_error = ik_step.metrics.error_norm;

    // Paden-Kahan subproblem 1: recover the angle of a quarter-turn about +z.
    // p and p_prime are equidistant from the axis, so the subproblem is
    // solvable (theta = pi/2). omega is the axis, q_axis a point on it.
    cartan::vector3<float> omega(0.f, 0.f, 1.f);
    cartan::vector3<float> q_axis(0.f, 0.f, 0.f);
    cartan::vector3<float> p(1.f, 0.f, 0.f);
    cartan::vector3<float> p_prime(0.f, 1.f, 0.f);
    auto pk1 = cartan::paden_kahan_1<float>(omega, q_axis, p, p_prime);
    const float pk1_theta = pk1.has_value() ? pk1.value() : 0.f;

    return fk_x + jac_trace + ik_error + pk1_theta;
}

}

/// Trivial entry point. Continuous integration compiles this file with `-c`
/// (no link step), so this never pulls in a runtime; it exists only to ODR-use
/// `app_stub()` and to route its result through a volatile sink so the whole
/// instantiation chain survives size optimization.
int main()
{
    volatile float sink = app_stub();
    return static_cast<int>(sink) & 0;
}
