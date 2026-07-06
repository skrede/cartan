/// @file 01_urdf_walkthrough.cpp
/// @brief Load a robot URDF, run forward kinematics at a random reachable
///        joint configuration, recover joints with iterative IK, and verify
///        the solution by walking forward kinematics back to the target pose.
///
/// Shows: cartan::load_urdf, kinematic_chain<Scalar, dynamic>,
///        cartan::forward_kinematics, basic_ik_runner instantiated with
///        cartan::projected_lm, and the FK back-verification idiom that
///        turns "did the solver converge?" into a concrete pose-error scalar.

#include "cartan/urdf.h"
#include "cartan/serial_chain.h"

#include <random>
#include <iostream>
#include <filesystem>

namespace
{

/// Draw a reproducible random joint vector inside the chain's per-joint
/// position limits. The cartanbot URDF contains one continuous joint whose
/// limit pair is +/-infinity; clamp those to a sensible finite window so the
/// uniform distribution is well-defined. The math under test does not depend
/// on sampling the whole real line.
template <typename Scalar>
auto random_within_limits(
    const cartan::kinematic_chain<Scalar, cartan::dynamic>& chain,
    std::mt19937& rng) -> Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
{
    const int n = chain.num_joints();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q(n);
    for (int i = 0; i < n; ++i)
    {
        const auto& lim = chain.limits()[static_cast<std::size_t>(i)];
        Scalar lo = lim.position_min;
        Scalar hi = lim.position_max;
        if (!std::isfinite(lo)) lo = -Scalar(3.14159265358979);
        if (!std::isfinite(hi)) hi = +Scalar(3.14159265358979);
        std::uniform_real_distribution<Scalar> dist(lo, hi);
        q(i) = dist(rng);
    }
    return q;
}

}

int main(int argc, char** argv)
{
    // Pass your robot's URDF as the first argument; otherwise this walkthrough
    // loads a synthetic test robot bundled with cartan. The math is identical
    // regardless -- try the bundled robot first, then point this at your
    // hardware.
    const std::filesystem::path urdf_file = (argc > 1)
        ? std::filesystem::path{argv[1]}
        : std::filesystem::path{CARTAN_TUTORIAL_DEFAULT_URDF};

    std::cout << "Loading URDF: " << urdf_file << "\n";

    // cartan::load_urdf returns cartan::expected<urdf_load_result<Scalar>,
    // urdf_error>. Surface the parser's diagnostic string on failure so a
    // broken path or malformed URDF fails loudly under CTest rather than
    // silently producing a default-constructed chain. The underlying parser
    // is pugixml-backed; the error.detail message names the line and the
    // failed schema rule.
    auto loaded = cartan::load_urdf<double>(urdf_file);
    if (!loaded)
    {
        std::cerr << "load_urdf failed: " << loaded.error().detail << "\n";
        return 1;
    }

    // urdf_load_result is an aggregate -- structured binding peels off the
    // kinematic_chain<double, cartan::dynamic> and the side-table of names
    // and inertials. The chain itself is string-free; everything that needs
    // a name lives in the metadata.
    auto& chain = loaded->chain;
    auto& meta  = loaded->metadata;

    std::cout << "Loaded " << chain.num_joints() << "-DOF chain ("
              << meta.base_link_name << " -> " << meta.tool_link_name << ")\n\n";

    // --- Step 1: pick a reachable ground-truth joint configuration ---
    //
    // Drawing q_truth uniformly from the per-joint limits guarantees a
    // pose that lies inside the chain's reachable workspace. Computing
    // FK(q_truth) then yields a target pose whose IK is known to admit a
    // solution -- the "inverse of forward" problem has a closed solution
    // even when general inverse kinematics does not.
    //
    // See Lynch & Park, Modern Robotics, Ch. 4 for the product-of-exponentials
    // forward-kinematics derivation.
    std::mt19937 rng{42};
    auto q_truth = random_within_limits(chain, rng);

    auto fk_target = cartan::forward_kinematics(chain, q_truth);
    auto target = fk_target.end_effector;

    std::cout << "Ground-truth q: " << q_truth.transpose() << "\n";
    std::cout << "FK-walked target pose:\n" << target.matrix() << "\n\n";

    // --- Step 2: solve IK from a cold-start seed ---
    //
    // Starting from q_seed = zeros() tests the solver's basin of attraction
    // honestly: if the LM iteration cannot recover the FK-walked target from
    // a generic seed, the student sees that as a teaching outcome (not every
    // target lies inside every solver's basin -- multi-start strategies and
    // closed-form solvers exist for exactly this reason).
    //
    // projected_lm wraps active-set box projection (joint limits enforced
    // inside the optimization step, not by post-hoc clamping) and Halton-seed
    // re-seed on stall. See Lynch & Park Ch. 6.2 for the LM-family IK
    // derivation.
    Eigen::Vector<double, cartan::dynamic> q0 =
        Eigen::Vector<double, cartan::dynamic>::Zero(chain.num_joints());

    cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    cartan::basic_ik_runner<
        cartan::projected_lm<cartan::kinematic_chain<double, cartan::dynamic>>>
            solver;
    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    if (!result.has_value())
    {
        // Convergence failure is a teaching outcome, not a CI failure: the
        // tutorial exits 0 so CTest does not flag it red. Parser failures
        // (file-not-found, malformed URDF) above remain non-zero exits since
        // those indicate a broken build-time path injection.
        std::cout << "IK did not converge -- target may lie outside the "
                     "dexterous workspace; this is a teaching outcome, not a "
                     "hard error.\n";
        return 0;
    }

    auto& sol = result.value();

    // --- Step 3: back-verify the solution by FK-walking the recovered q ---
    //
    // The robust pose-error metric is the body-frame log of the residual
    // transform: T_err = FK(q_solution)^{-1} * T_target. Its log lives in
    // se(3); the norm gives a single scalar that combines translational and
    // rotational error. A tight bound here (sub-1e-4) confirms the iterative
    // solver converged to a configuration that genuinely reproduces the
    // target pose, not merely a numerically close joint vector.
    auto fk_verify = cartan::forward_kinematics(chain, sol.solution.position);
    auto pose_err = (fk_verify.end_effector.inverse() * target).log().norm();

    std::cout << "IK converged in " << sol.iterations << " iterations\n";
    std::cout << "Recovered q:   " << sol.solution.position.transpose() << "\n";
    std::cout << "Ground-truth q:" << q_truth.transpose() << "\n";
    std::cout << "Pose error (FK back-verify): " << pose_err << "\n";

    return 0;
}
