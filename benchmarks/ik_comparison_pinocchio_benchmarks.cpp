/// @file ik_comparison_pinocchio_benchmarks.cpp
/// @brief Head-to-head IK: cartan-LM vs pinocchio-LM over identical targets.
///
/// pinocchio-LM is a hand-rolled mirror of cartan::ik::lm: body-frame error
/// V_b = log(T_curr^{-1} * T_target), body Jacobian, Nielsen damping update,
/// LDLT solve of (J^T J + lambda I) dq = J^T V_b. Same 10000 target/seed pairs
/// (seed 42), same convergence criteria (1e-5 / 1e-5), same iteration cap (100).
///
/// Both solvers see the same problem set; the wall-time and accuracy gap is
/// purely the algorithm-equal cost of FK + Jacobian + linear-solve per step
/// in the two libraries' representations.

#include "benchmark_utils.h"

#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/solver/lm.h>
#include <cartan/serial/ik/solver/dls.h>
#include <cartan/serial/ik/solver/lbfgsb.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/ik/basic_ik_runner.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/ik/solver/projected_lm.h>
#include <cartan/serial/ik/policy/limits_policy.h>
#include <cartan/serial/ik/solver/newton_raphson.h>
#include <cartan/serial/ik/wrapper/restart_wrapper.h>

#ifdef CARTAN_BUILD_ARGMIN
#include <cartan/serial/ik/solver/argmin_lm.h>
#include <cartan/serial/ik/solver/argmin_slsqp.h>
#include <cartan/serial/ik/solver/argmin_bobyqa.h>
#include <cartan/serial/ik/solver/argmin_lbfgsb.h>
#include <cartan/serial/ik/solver/argmin_projected_gn.h>
#include <cartan/serial/ik/solver/argmin_projected_gradient_gn.h>
#endif

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

#ifdef CARTAN_HAS_TRAC_IK
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#endif

#include <benchmark/benchmark.h>

#include <Eigen/Dense>

#include <random>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

// ============================================================================
// Per-family algorithmic-work-unit budgets
// ============================================================================
//
// One algorithmic-work unit = one major iteration of the solver's design. The
// runner caps total work at `max_total_work_units` (literally, no slack); each
// solver's internal per-attempt counter caps a single attempt at
// `max_iterations_per_attempt`.
//
// Initial values match the pre-refactor effective compute envelope
// (`max_iter * 2`, where the historic runner slack factor 2x was dropped at the
// runner cap). Each cell's restart variant shares the per-family total because
// restart_wrapper's pass-through unit aggregation means a single per-family
// number applies to both the default and the restart-wrapped variant. These are
// initial values — to be validated empirically against the cross-solver bench
// baseline in a follow-up.

namespace cartan::ik::bench
{

// LM family: 1 unit = one LM iteration (form normal eqs + LDLT solve +
// accept/reject step). Pre-refactor used max_iter=100 for default cells and
// max_iter=200 for the _restart cell. Honest uniform per-family total is
// empirically sized at 800: a ladder walk (400/800/1600/3200) showed every
// LM cell reaches its natural convergence ceiling by 800 and is bit-identical
// at higher values, so 800 is the smallest value that does not truncate any
// cell. Wall time is insensitive to the cap past 800 because saturated cells
// terminate on convergence, not on budget.
constexpr int lm_family_total_units = 800;

// L-BFGS-B family: 1 unit = one Armijo line search (cartan) or one argmin-internal
// L-BFGS-B iteration (argmin shim). Same pre-refactor sizing as LM.
constexpr int lbfgsb_family_total_units = 400;

// SQP / SLSQP family: 1 unit = one argmin-internal SLSQP outer iteration.
// Per-iter cost is significantly higher than LM (each iter is a full QP).
// Pre-refactor used max_iter=50/100 for default/restart. Effective = 100/200.
// Honest uniform per-family total = 200 units.
constexpr int sqp_family_total_units = 200;

// BOBYQA family: 1 unit = one argmin-internal BOBYQA iteration. Derivative-free,
// comparable cost to SLSQP per iter. Honest uniform per-family total = 200.
constexpr int bobyqa_family_total_units = 200;

// Projected GN family: 1 unit = one argmin-internal outer GN iteration.
// Pre-refactor max_iter=100/200. Honest uniform per-family total = 400.
constexpr int projected_gn_family_total_units = 400;

// Newton-Raphson family: 1 unit = one Newton iteration. Pre-refactor max_iter=100.
// Honest uniform per-family total = 200.
constexpr int newton_family_total_units = 200;

// Projected LM (self-restarting): 1 unit = one outer projected-LM iteration.
// Pre-refactor max_iter=100/200. Halton-reseed restart events charge 0 units;
// the budget is the inner iteration count across all internal restarts.
constexpr int projected_lm_total_units = 400;

// Per-attempt cap is uniform across families. Matches pre-refactor default
// max_iter; each solver's internal m_iterations counter checks against this.
constexpr int per_family_per_attempt = 100;

}  // namespace cartan::ik::bench

namespace
{

constexpr int num_targets = 10000;

// ============================================================================
// Target set generation — identical seed/protocol to ik_comparison_benchmarks
// ============================================================================

template <typename Scalar, int N>
struct target_set
{
    using position_type = typename cartan::joint_state<Scalar, N>::position_type;
    std::vector<cartan::se3<Scalar>> targets;
    std::vector<position_type> seeds;

    target_set(const cartan::kinematic_chain<Scalar, N>& chain, int count, unsigned seed = 42)
    {
        std::mt19937 rng(seed);
        targets.reserve(static_cast<std::size_t>(count));
        seeds.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i)
        {
            targets.push_back(cartan::benchmarks::random_reachable_target(chain, rng));
            seeds.push_back(cartan::benchmarks::random_joint_config(chain, rng));
        }
    }
};

// ============================================================================
// Cartan PoE -> Pinocchio Model converter (mirror of fk_pinocchio_benchmarks)
// ============================================================================

struct pinocchio_chain
{
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex ee_frame_id{};

    explicit pinocchio_chain(pinocchio::Model m)
        : model(std::move(m)), data(model)
    {
    }
};

template <typename Chain>
pinocchio_chain build_pinocchio_chain(const Chain& chain, const std::string& name)
{
    using Scalar = typename Chain::scalar_type;
    using vec3 = Eigen::Matrix<Scalar, 3, 1>;

    pinocchio::Model model;
    model.name = name;

    pinocchio::JointIndex parent_id = 0;
    vec3 prev_axis_point = vec3::Zero();

    const int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        const auto& s = chain.axis(i);
        if (!s.is_revolute())
        {
            throw std::runtime_error("pinocchio chain builder: prismatic joints not supported");
        }
        vec3 omega = s.omega();
        vec3 v = s.v();
        vec3 q_perp = omega.cross(v);

        pinocchio::SE3 placement;
        placement.setIdentity();
        placement.translation() = q_perp - prev_axis_point;

        parent_id = model.addJoint(
            parent_id,
            pinocchio::JointModelRevoluteUnaligned(omega.template cast<double>()),
            placement,
            "j" + std::to_string(i));

        prev_axis_point = q_perp;
    }

    const auto& home = chain.home();
    pinocchio::SE3 ee_placement;
    ee_placement.rotation() = home.rotation().matrix().template cast<double>();
    ee_placement.translation() = home.translation().template cast<double>() - prev_axis_point.template cast<double>();

    pinocchio_chain pc(std::move(model));
    pc.ee_frame_id = pc.model.addFrame(pinocchio::Frame(
        "ee", parent_id, ee_placement, pinocchio::OP_FRAME));
    pc.data = pinocchio::Data(pc.model);
    return pc;
}

// ============================================================================
// Pinocchio LM step (mirror of cartan::ik::lm algorithm)
// ============================================================================
//
// Mirrors cartan-LM exactly:
//   V_b   = log6(T_curr^{-1} * T_target)            (body twist error)
//   J_b   = body Jacobian at q                      (LOCAL frame in pinocchio)
//   H     = J_b^T J_b
//   g     = J_b^T V_b
//   dq    = (H + lambda I)^{-1} g                   (LDLT)
//   rho   = (||V_b||^2 - ||V_b_trial||^2) / dq^T (lambda dq + g)
//   if rho > 0: accept, lambda *= max(1/3, 1 - (2 rho - 1)^3)
//   else:       lambda *= nu;  nu *= 2
// init: lambda = 1e-3 * max_diag(H);  nu = 2;  max_iter = 100;  tol = 1e-5
//
// Returns (success, iterations, q_final, V_b_norm).

struct pin_ik_result
{
    bool converged{false};
    int iterations{0};
    Eigen::VectorXd q;
    double pos_err{0.0};
    double ori_err{0.0};
};

inline pin_ik_result pinocchio_lm_solve(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    pinocchio::FrameIndex ee_frame_id,
    const pinocchio::SE3& target,
    Eigen::VectorXd q,
    int max_iter = 100,
    double tol = 1e-5)
{
    const int n = model.nv;
    pin_ik_result r;
    r.q = q;

    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::SE3 T_curr = data.oMf[ee_frame_id];
    Eigen::Matrix<double, 6, 1> V_b = pinocchio::log6(T_curr.actInv(target)).toVector();
    double err_old_sq = V_b.squaredNorm();

    Eigen::MatrixXd J_b = Eigen::MatrixXd::Zero(6, n);
    pinocchio::computeFrameJacobian(model, data, q, ee_frame_id,
                                    pinocchio::LOCAL, J_b);
    Eigen::MatrixXd H = J_b.transpose() * J_b;
    double max_diag = 0.0;
    for (int i = 0; i < n; ++i) max_diag = std::max(max_diag, H(i, i));
    double lambda = 1e-3 * max_diag;
    if (lambda < std::numeric_limits<double>::epsilon()) lambda = 1e-4;
    double nu = 2.0;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // pinocchio Motion vector layout is [linear; angular] (head=v, tail=omega)
        double lin = V_b.template head<3>().norm();
        double ang = V_b.template tail<3>().norm();
        if (ang < tol && lin < tol)
        {
            r.converged = true;
            r.iterations = iter;
            r.q = q;
            r.ori_err = ang;
            r.pos_err = lin;
            return r;
        }

        // Recompute J + H + g at current q (cartan does this each step)
        pinocchio::computeFrameJacobian(model, data, q, ee_frame_id,
                                        pinocchio::LOCAL, J_b);
        H.noalias() = J_b.transpose() * J_b;
        Eigen::VectorXd g = J_b.transpose() * V_b;

        Eigen::MatrixXd A = H;
        for (int i = 0; i < n; ++i) A(i, i) += lambda;

        Eigen::VectorXd dq = A.ldlt().solve(g);
        Eigen::VectorXd q_trial = q + dq;

        pinocchio::framesForwardKinematics(model, data, q_trial);
        pinocchio::SE3 T_trial = data.oMf[ee_frame_id];
        Eigen::Matrix<double, 6, 1> V_b_trial =
            pinocchio::log6(T_trial.actInv(target)).toVector();
        double err_new_sq = V_b_trial.squaredNorm();

        double pred_red = dq.transpose() * (lambda * dq + g);
        double rho = 0.0;
        if (std::abs(pred_red) > std::numeric_limits<double>::epsilon())
            rho = (err_old_sq - err_new_sq) / pred_red;

        if (rho > 0.0)
        {
            q = q_trial;
            V_b = V_b_trial;
            err_old_sq = err_new_sq;
            double factor = 1.0 - std::pow(2.0 * rho - 1.0, 3.0);
            lambda *= std::max(1.0 / 3.0, factor);
            nu = 2.0;
        }
        else
        {
            lambda *= nu;
            nu *= 2.0;
        }
    }

    r.iterations = max_iter;
    r.q = q;
    r.pos_err = V_b.template head<3>().norm();
    r.ori_err = V_b.template tail<3>().norm();
    r.converged = (r.ori_err < tol && r.pos_err < tol);
    return r;
}

// ============================================================================
// Bench drivers
// ============================================================================

template <int N>
void bm_cartan_lm(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    cartan::convergence_criteria<double> criteria{
        .position_tol = 1e-5,
        .orientation_tol = 1e-5,
        .max_iterations_per_attempt = 100,
        .max_total_work_units = 200};

    std::size_t idx = 0;
    int successes = 0;
    int total_iter = 0;
    double total_pos = 0.0, total_ori = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::basic_ik_runner<cartan::ik::lm<cartan::kinematic_chain<double, N>>> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iter += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos += pos_err;
            total_ori += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1));
    state.counters["avg_iter"] = benchmark::Counter(
        static_cast<double>(total_iter) / std::max(successes, 1));
    state.counters["pos_err"] = benchmark::Counter(
        total_pos / std::max(successes, 1));
    state.counters["ori_err"] = benchmark::Counter(
        total_ori / std::max(successes, 1));
}

template <int N>
void bm_cartan_restart_lm(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts)
{
    using chain_t = cartan::kinematic_chain<double, N>;
    using restart_lm = cartan::ik::restart_wrapper<chain_t, cartan::ik::lm<chain_t>>;
    cartan::convergence_criteria<double> criteria{
        .position_tol = 1e-5,
        .orientation_tol = 1e-5,
        .max_iterations_per_attempt = 200,
        .max_total_work_units = 200};

    std::size_t idx = 0;
    int successes = 0;
    int total_iter = 0;
    double total_pos = 0.0, total_ori = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::basic_ik_runner<restart_lm> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iter += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos += pos_err;
            total_ori += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1));
    state.counters["avg_iter"] = benchmark::Counter(
        static_cast<double>(total_iter) / std::max(successes, 1));
    state.counters["pos_err"] = benchmark::Counter(
        total_pos / std::max(successes, 1));
    state.counters["ori_err"] = benchmark::Counter(
        total_ori / std::max(successes, 1));
}

// Generalized driver: any Solver type satisfying the cartan::ik::solve_policy concept.
// Used by IK_BENCH_SOLVER_VARIANTS to emit {default, _no_limits, _restart} cells per
// base solver per robot. The driver takes two int parameters: `per_attempt` caps a
// single solver attempt (consulted by every solver's internal iteration counter and
// by self-restarting solvers as their per-attempt budget before triggering a restart);
// `total_units` is the runner-level total budget measured in algorithmic work units
// (1 unit = one major iteration of the solver's design).
template <int N, typename Solver>
void bm_cartan_solver(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts,
    int per_attempt,
    int total_units)
{
    cartan::convergence_criteria<double> criteria{
        .position_tol = 1e-5,
        .orientation_tol = 1e-5,
        .max_iterations_per_attempt = per_attempt,
        .max_total_work_units = total_units};

    std::size_t idx = 0;
    int successes = 0;
    int total_iter = 0;
    double total_pos = 0.0, total_ori = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::basic_ik_runner<Solver> solver;
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iter += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos += pos_err;
            total_ori += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1));
    state.counters["avg_iter"] = benchmark::Counter(
        static_cast<double>(total_iter) / std::max(successes, 1));
    state.counters["pos_err"] = benchmark::Counter(
        total_pos / std::max(successes, 1));
    state.counters["ori_err"] = benchmark::Counter(
        total_ori / std::max(successes, 1));
}

#ifdef CARTAN_BUILD_ARGMIN
// k-sweep driver: same shape as bm_cartan_solver but constructs the inner
// argmin_slsqp solver with options.multiplier_reest_every_k = K so the
// post-step active-set Lagrange multiplier re-estimation stride is forced
// to the swept value (overriding kraft_slsqp_policy's per-Mode default).
//
// The Solver type is expected to be argmin_slsqp<Chain, LimitsPolicy, Conv, Mode>
// (or one of its aliases like argmin_slsqp_fast) — any solver whose options
// struct carries a multiplier_reest_every_k field. The driver wraps the
// solver in a basic_ik_runner the same way bm_cartan_solver does, so the
// per-pose / per-attempt / total-budget semantics match the existing
// argmin_slsqp cells exactly.
template <int N, typename Solver>
void bm_cartan_argmin_slsqp_kreest(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const target_set<double, N>& ts,
    int per_attempt,
    int total_units,
    std::size_t k_value)
{
    cartan::convergence_criteria<double> criteria{
        .position_tol = 1e-5,
        .orientation_tol = 1e-5,
        .max_iterations_per_attempt = per_attempt,
        .max_total_work_units = total_units};

    typename Solver::options slsqp_opts{};
    slsqp_opts.multiplier_reest_every_k = k_value;

    std::size_t idx = 0;
    int successes = 0;
    int total_iter = 0;
    double total_pos = 0.0, total_ori = 0.0;

    for (auto _ : state)
    {
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];
        ++idx;

        cartan::basic_ik_runner<Solver> solver{Solver{slsqp_opts}};
        solver.setup(chain, target, q_seed, criteria);
        auto result = solver.solve();

        if (result.has_value())
        {
            ++successes;
            total_iter += result->iterations;
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, result->solution.position, target);
            total_pos += pos_err;
            total_ori += ori_err;
        }
        benchmark::DoNotOptimize(result);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1));
    state.counters["avg_iter"] = benchmark::Counter(
        static_cast<double>(total_iter) / std::max(successes, 1));
    state.counters["pos_err"] = benchmark::Counter(
        total_pos / std::max(successes, 1));
    state.counters["ori_err"] = benchmark::Counter(
        total_ori / std::max(successes, 1));
    state.counters["k_reest"] = benchmark::Counter(static_cast<double>(k_value));
}
#endif

template <int N>
void bm_pinocchio_lm(
    benchmark::State& state,
    const pinocchio::Model& model_template,
    pinocchio::FrameIndex ee_frame_id,
    const target_set<double, N>& ts)
{
    pinocchio::Model model = model_template;
    pinocchio::Data data(model);

    // Pre-convert targets and seeds to pinocchio types
    auto count = static_cast<std::size_t>(num_targets);
    std::vector<pinocchio::SE3> pin_targets;
    std::vector<Eigen::VectorXd> pin_seeds;
    pin_targets.reserve(count);
    pin_seeds.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        pinocchio::SE3 T;
        T.rotation() = ts.targets[i].rotation().matrix();
        T.translation() = ts.targets[i].translation();
        pin_targets.push_back(T);

        Eigen::VectorXd q(N);
        for (int j = 0; j < N; ++j) q(j) = ts.seeds[i](j);
        pin_seeds.push_back(q);
    }

    std::size_t idx = 0;
    int successes = 0;
    int total_iter = 0;
    double total_pos = 0.0, total_ori = 0.0;

    for (auto _ : state)
    {
        const auto& target = pin_targets[idx % count];
        const auto& q_seed = pin_seeds[idx % count];
        ++idx;

        auto r = pinocchio_lm_solve(model, data, ee_frame_id, target, q_seed);
        if (r.converged)
        {
            ++successes;
            total_iter += r.iterations;
            total_pos += r.pos_err;
            total_ori += r.ori_err;
        }
        benchmark::DoNotOptimize(r);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(successes) / std::max(total, 1));
    state.counters["avg_iter"] = benchmark::Counter(
        static_cast<double>(total_iter) / std::max(successes, 1));
    state.counters["pos_err"] = benchmark::Counter(
        total_pos / std::max(successes, 1));
    state.counters["ori_err"] = benchmark::Counter(
        total_ori / std::max(successes, 1));
}

#ifdef CARTAN_HAS_TRAC_IK
// TRAC-IK with external verification of returned q against the SAME tolerance
// gate cartan and pinocchio use. The CartToJnt return code alone is unreliable
// — it can return rc>=0 for solutions outside tolerance.
template <int N>
void bm_trac_ik_verified(
    benchmark::State& state,
    const cartan::kinematic_chain<double, N>& chain,
    const KDL::Chain& kdl_chain,
    KDL::JntArray q_min,
    KDL::JntArray q_max,
    const target_set<double, N>& ts)
{
    auto count = static_cast<std::size_t>(num_targets);
    std::vector<KDL::Frame> kdl_targets;
    std::vector<KDL::JntArray> kdl_seeds;
    kdl_targets.reserve(count);
    kdl_seeds.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        kdl_targets.push_back(cartan::benchmarks::se3_to_kdl_frame(ts.targets[i]));
        KDL::JntArray seed(static_cast<unsigned int>(N));
        for (unsigned int j = 0; j < static_cast<unsigned int>(N); ++j)
            seed(j) = ts.seeds[i](static_cast<int>(j));
        kdl_seeds.push_back(seed);
    }

    constexpr double tol = 1e-5;
    TRAC_IK::TRAC_IK solver(kdl_chain, q_min, q_max,
                             /*maxtime=*/10.0, /*eps=*/tol, TRAC_IK::Speed);

    std::size_t idx = 0;
    int rc_ok = 0;
    int verified_ok = 0;
    double total_pos = 0.0, total_ori = 0.0;

    using position_type = typename cartan::joint_state<double, N>::position_type;

    for (auto _ : state)
    {
        std::size_t cur = idx % count;
        auto& target = kdl_targets[cur];
        auto& q_init = kdl_seeds[cur];
        ++idx;

        KDL::JntArray q_out(static_cast<unsigned int>(N));
        int rc = solver.CartToJnt(q_init, target, q_out);

        if (rc >= 0)
        {
            ++rc_ok;
            // External verification: cartan FK + body twist error vs target
            position_type q_cartan;
            for (int j = 0; j < N; ++j) q_cartan(j) = q_out(static_cast<unsigned int>(j));
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(
                chain, q_cartan, ts.targets[cur]);
            if (ori_err < tol && pos_err < tol)
            {
                ++verified_ok;
                total_pos += pos_err;
                total_ori += ori_err;
            }
        }
        benchmark::DoNotOptimize(q_out);
    }

    auto total = static_cast<int>(idx);
    state.counters["Success_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(verified_ok) / std::max(total, 1));
    state.counters["rc_pct"] = benchmark::Counter(
        100.0 * static_cast<double>(rc_ok) / std::max(total, 1));
    state.counters["pos_err"] = benchmark::Counter(
        total_pos / std::max(verified_ok, 1));
    state.counters["ori_err"] = benchmark::Counter(
        total_ori / std::max(verified_ok, 1));
}
#endif

// ============================================================================
// Per-robot drivers (registered at static init)
// ============================================================================

#ifdef CARTAN_HAS_TRAC_IK
#define IK_BENCH_ROBOT_TRAC_IK(ROBOT, FACTORY, KDL_FACTORY, KDL_LIMITS_FACTORY, N_DOF)  \
static void bm_ik_##ROBOT##_trac_ik(benchmark::State& state)                            \
{                                                                                       \
    static auto chain = cartan::benchmarks::FACTORY<double>();                          \
    static target_set<double, N_DOF> ts(chain, num_targets);                            \
    static auto kdl_chain = cartan::benchmarks::KDL_FACTORY();                          \
    KDL::JntArray q_min(N_DOF), q_max(N_DOF);                                           \
    cartan::benchmarks::KDL_LIMITS_FACTORY(q_min, q_max);                               \
    bm_trac_ik_verified<N_DOF>(state, chain, kdl_chain, q_min, q_max, ts);              \
}                                                                                       \
BENCHMARK(bm_ik_##ROBOT##_trac_ik)->Iterations(2000)->Unit(benchmark::kMicrosecond)
#else
#define IK_BENCH_ROBOT_TRAC_IK(ROBOT, FACTORY, KDL_FACTORY, KDL_LIMITS_FACTORY, N_DOF) /* trac_ik disabled */
#endif

#define IK_BENCH_ROBOT(ROBOT, FACTORY, KDL_FACTORY, KDL_LIMITS_FACTORY, N_DOF)          \
static void bm_ik_##ROBOT##_cartan_lm(benchmark::State& state)                          \
{                                                                                       \
    static auto chain = cartan::benchmarks::FACTORY<double>();                          \
    static target_set<double, N_DOF> ts(chain, num_targets);                            \
    bm_cartan_lm<N_DOF>(state, chain, ts);                                              \
}                                                                                       \
BENCHMARK(bm_ik_##ROBOT##_cartan_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond);  \
static void bm_ik_##ROBOT##_cartan_restart_lm(benchmark::State& state)                  \
{                                                                                       \
    static auto chain = cartan::benchmarks::FACTORY<double>();                          \
    static target_set<double, N_DOF> ts(chain, num_targets);                            \
    bm_cartan_restart_lm<N_DOF>(state, chain, ts);                                      \
}                                                                                       \
BENCHMARK(bm_ik_##ROBOT##_cartan_restart_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond); \
static void bm_ik_##ROBOT##_pinocchio_lm(benchmark::State& state)                       \
{                                                                                       \
    static auto chain = cartan::benchmarks::FACTORY<double>();                          \
    static target_set<double, N_DOF> ts(chain, num_targets);                            \
    static auto pc = build_pinocchio_chain(chain, #ROBOT);                              \
    bm_pinocchio_lm<N_DOF>(state, pc.model, pc.ee_frame_id, ts);                        \
}                                                                                       \
BENCHMARK(bm_ik_##ROBOT##_pinocchio_lm)->Iterations(2000)->Unit(benchmark::kMicrosecond); \
IK_BENCH_ROBOT_TRAC_IK(ROBOT, FACTORY, KDL_FACTORY, KDL_LIMITS_FACTORY, N_DOF)

// Per-solver macro: emits three cells per (ROBOT, SOLVER) pair —
//   bm_ik_<robot>_<solver>           (default LimitsPolicy)
//   bm_ik_<robot>_<solver>_no_limits (audit cell, explicit cartan::no_limits)
//   bm_ik_<robot>_<solver>_restart   (restart_wrapper around default-flavored solver)
//
// SOLVER_NAME is the bare identifier used in the cell name (e.g. dls).
// SOLVER_TPL is the qualified solver template (e.g. cartan::ik::dls).
// FAMILY_TOTAL_UNITS names the per-family `constexpr int` budget constant
// defined in the `cartan::ik::bench` namespace above; the same constant is
// threaded into both the default-flavored cell and the restart-wrapped cell
// because restart_wrapper's pass-through unit aggregation means no per-cell
// wrapper-specific tuning is required.
// The macro relies on bm_cartan_solver<N, Solver> being defined in the same TU.
#define IK_BENCH_SOLVER_VARIANTS(ROBOT, FACTORY, N_DOF, SOLVER_NAME, SOLVER_TPL, FAMILY_TOTAL_UNITS) \
static void bm_ik_##ROBOT##_##SOLVER_NAME(benchmark::State& state)                       \
{                                                                                        \
    using chain_t = cartan::kinematic_chain<double, N_DOF>;                              \
    static auto chain = cartan::benchmarks::FACTORY<double>();                           \
    static target_set<double, N_DOF> ts(chain, num_targets);                             \
    bm_cartan_solver<N_DOF, SOLVER_TPL<chain_t>>(                                        \
        state, chain, ts,                                                                \
        cartan::ik::bench::per_family_per_attempt,                                       \
        (FAMILY_TOTAL_UNITS));                                                           \
}                                                                                        \
BENCHMARK(bm_ik_##ROBOT##_##SOLVER_NAME)->Iterations(2000)->Unit(benchmark::kMicrosecond); \
static void bm_ik_##ROBOT##_##SOLVER_NAME##_no_limits(benchmark::State& state)           \
{                                                                                        \
    using chain_t = cartan::kinematic_chain<double, N_DOF>;                              \
    static auto chain = cartan::benchmarks::FACTORY<double>();                           \
    static target_set<double, N_DOF> ts(chain, num_targets);                             \
    bm_cartan_solver<N_DOF, SOLVER_TPL<chain_t, cartan::no_limits>>(                     \
        state, chain, ts,                                                                \
        cartan::ik::bench::per_family_per_attempt,                                       \
        (FAMILY_TOTAL_UNITS));                                                           \
}                                                                                        \
BENCHMARK(bm_ik_##ROBOT##_##SOLVER_NAME##_no_limits)->Iterations(2000)->Unit(benchmark::kMicrosecond); \
static void bm_ik_##ROBOT##_##SOLVER_NAME##_restart(benchmark::State& state)             \
{                                                                                        \
    using chain_t = cartan::kinematic_chain<double, N_DOF>;                              \
    static auto chain = cartan::benchmarks::FACTORY<double>();                           \
    static target_set<double, N_DOF> ts(chain, num_targets);                             \
    bm_cartan_solver<N_DOF,                                                              \
        cartan::ik::restart_wrapper<chain_t, SOLVER_TPL<chain_t>>>(                      \
        state, chain, ts,                                                                \
        cartan::ik::bench::per_family_per_attempt,                                       \
        (FAMILY_TOTAL_UNITS));                                                           \
}                                                                                        \
BENCHMARK(bm_ik_##ROBOT##_##SOLVER_NAME##_restart)->Iterations(2000)->Unit(benchmark::kMicrosecond)

// argmin_slsqp k-sweep macro: emits three cells per (ROBOT, K_VALUE) pair —
//   bm_ik_<robot>_argmin_slsqp_fast_kreest_k<K_VALUE>           (default LimitsPolicy)
//   bm_ik_<robot>_argmin_slsqp_fast_kreest_k<K_VALUE>_no_limits (audit cell)
//   bm_ik_<robot>_argmin_slsqp_fast_kreest_k<K_VALUE>_restart   (restart-wrapped)
//
// Same {default, no_limits, restart} variant axis as IK_BENCH_SOLVER_VARIANTS, but
// the underlying solver is constructed with options.multiplier_reest_every_k = K_VALUE
// (overriding kraft_slsqp_policy's per-Mode default of 5 on sqp_mode::fast).
// SOLVER_TPL is expected to be cartan::ik::argmin_slsqp_fast (the alias resolving
// argmin::sqp_mode::fast through kraft_slsqp_policy).
//
// Cell names encode the k axis in the cell-name suffix so --benchmark_filter
// regex selection (`kreest_k`) picks up the whole sweep.
#define IK_BENCH_ARGMIN_SLSQP_KREEST_VARIANTS(ROBOT, FACTORY, N_DOF, SOLVER_NAME, SOLVER_TPL, FAMILY_TOTAL_UNITS, K_VALUE) \
static void bm_ik_##ROBOT##_##SOLVER_NAME##_kreest_k##K_VALUE(benchmark::State& state)              \
{                                                                                                   \
    using chain_t = cartan::kinematic_chain<double, N_DOF>;                                         \
    static auto chain = cartan::benchmarks::FACTORY<double>();                                      \
    static target_set<double, N_DOF> ts(chain, num_targets);                                        \
    bm_cartan_argmin_slsqp_kreest<N_DOF, SOLVER_TPL<chain_t>>(                                      \
        state, chain, ts,                                                                           \
        cartan::ik::bench::per_family_per_attempt,                                                  \
        (FAMILY_TOTAL_UNITS),                                                                       \
        static_cast<std::size_t>(K_VALUE));                                                         \
}                                                                                                   \
BENCHMARK(bm_ik_##ROBOT##_##SOLVER_NAME##_kreest_k##K_VALUE)->Iterations(2000)->Unit(benchmark::kMicrosecond); \
static void bm_ik_##ROBOT##_##SOLVER_NAME##_kreest_k##K_VALUE##_no_limits(benchmark::State& state)  \
{                                                                                                   \
    using chain_t = cartan::kinematic_chain<double, N_DOF>;                                         \
    static auto chain = cartan::benchmarks::FACTORY<double>();                                      \
    static target_set<double, N_DOF> ts(chain, num_targets);                                        \
    bm_cartan_argmin_slsqp_kreest<N_DOF, SOLVER_TPL<chain_t, cartan::no_limits>>(                   \
        state, chain, ts,                                                                           \
        cartan::ik::bench::per_family_per_attempt,                                                  \
        (FAMILY_TOTAL_UNITS),                                                                       \
        static_cast<std::size_t>(K_VALUE));                                                         \
}                                                                                                   \
BENCHMARK(bm_ik_##ROBOT##_##SOLVER_NAME##_kreest_k##K_VALUE##_no_limits)->Iterations(2000)->Unit(benchmark::kMicrosecond); \
static void bm_ik_##ROBOT##_##SOLVER_NAME##_kreest_k##K_VALUE##_restart(benchmark::State& state)    \
{                                                                                                   \
    using chain_t = cartan::kinematic_chain<double, N_DOF>;                                         \
    using inner_t = SOLVER_TPL<chain_t>;                                                            \
    using wrapped_t = cartan::ik::restart_wrapper<chain_t, inner_t>;                                \
    static auto chain = cartan::benchmarks::FACTORY<double>();                                      \
    static target_set<double, N_DOF> ts(chain, num_targets);                                        \
    /* Restart-wrapped variant: construct the inner solver explicitly with k-override, */          \
    /* then wrap it. Same shape as bm_cartan_argmin_slsqp_kreest but Solver=wrapped_t. */           \
    cartan::convergence_criteria<double> criteria{                                                  \
        .position_tol = 1e-5,                                                                       \
        .orientation_tol = 1e-5,                                                                    \
        .max_iterations_per_attempt = cartan::ik::bench::per_family_per_attempt,                    \
        .max_total_work_units = (FAMILY_TOTAL_UNITS)};                                              \
    typename inner_t::options slsqp_opts{};                                                         \
    slsqp_opts.multiplier_reest_every_k = static_cast<std::size_t>(K_VALUE);                        \
    std::size_t idx = 0;                                                                            \
    int successes = 0;                                                                              \
    int total_iter = 0;                                                                             \
    double total_pos = 0.0, total_ori = 0.0;                                                        \
    for (auto _ : state) {                                                                          \
        auto& target = ts.targets[idx % static_cast<std::size_t>(num_targets)];                     \
        auto& q_seed = ts.seeds[idx % static_cast<std::size_t>(num_targets)];                       \
        ++idx;                                                                                      \
        cartan::basic_ik_runner<wrapped_t> solver{wrapped_t{inner_t{slsqp_opts}}};                  \
        solver.setup(chain, target, q_seed, criteria);                                              \
        auto result = solver.solve();                                                               \
        if (result.has_value()) {                                                                   \
            ++successes;                                                                            \
            total_iter += result->iterations;                                                       \
            auto [pos_err, ori_err] = cartan::benchmarks::compute_pose_errors(                      \
                chain, result->solution.position, target);                                          \
            total_pos += pos_err;                                                                   \
            total_ori += ori_err;                                                                   \
        }                                                                                           \
        benchmark::DoNotOptimize(result);                                                           \
    }                                                                                               \
    auto total = static_cast<int>(idx);                                                             \
    state.counters["Success_pct"] = benchmark::Counter(                                             \
        100.0 * static_cast<double>(successes) / std::max(total, 1));                               \
    state.counters["avg_iter"] = benchmark::Counter(                                                \
        static_cast<double>(total_iter) / std::max(successes, 1));                                  \
    state.counters["pos_err"] = benchmark::Counter(total_pos / std::max(successes, 1));             \
    state.counters["ori_err"] = benchmark::Counter(total_ori / std::max(successes, 1));             \
    state.counters["k_reest"] = benchmark::Counter(static_cast<double>(K_VALUE));                   \
}                                                                                                   \
BENCHMARK(bm_ik_##ROBOT##_##SOLVER_NAME##_kreest_k##K_VALUE##_restart)->Iterations(2000)->Unit(benchmark::kMicrosecond)

// Single-cell variant for solvers whose public template already encapsulates restart logic
// (e.g. projected_lm self-restarts on stall via Halton re-seed). Adding _no_limits and
// _restart variants for such solvers would emit a redundant duplicate of the default cell
// and a pathological double-restart cell respectively, so only the default is registered.
#define IK_BENCH_DEFAULT_ONLY(ROBOT, FACTORY, N_DOF, SOLVER_NAME, SOLVER_TPL, FAMILY_TOTAL_UNITS) \
static void bm_ik_##ROBOT##_##SOLVER_NAME(benchmark::State& state)                       \
{                                                                                        \
    using chain_t = cartan::kinematic_chain<double, N_DOF>;                              \
    static auto chain = cartan::benchmarks::FACTORY<double>();                           \
    static target_set<double, N_DOF> ts(chain, num_targets);                             \
    bm_cartan_solver<N_DOF, SOLVER_TPL<chain_t>>(                                        \
        state, chain, ts,                                                                \
        cartan::ik::bench::per_family_per_attempt,                                       \
        (FAMILY_TOTAL_UNITS));                                                           \
}                                                                                        \
BENCHMARK(bm_ik_##ROBOT##_##SOLVER_NAME)->Iterations(2000)->Unit(benchmark::kMicrosecond)

IK_BENCH_ROBOT(ur3e,       make_ur3e_chain,       make_ur3e_kdl_chain,       make_ur3e_kdl_limits,       6);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(ur3e,       make_ur3e_chain,       6, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(kr6_sixx,   make_kr6_sixx_chain,   make_kr6_sixx_kdl_chain,   make_kr6_sixx_kdl_limits,   6);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(kr6_sixx,   make_kr6_sixx_chain,   6, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(abb_irb120, make_abb_irb120_chain, make_abb_irb120_kdl_chain, make_abb_irb120_kdl_limits, 6);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(abb_irb120, make_abb_irb120_chain, 6, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(jaco2,      make_jaco2_chain,      make_jaco2_kdl_chain,      make_jaco2_kdl_limits,      6);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(jaco2,      make_jaco2_chain,      6, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(lbr_med14,  make_lbr_med14_chain,  make_lbr_med14_kdl_chain,  make_lbr_med14_kdl_limits,  7);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(lbr_med14,  make_lbr_med14_chain,  7, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(panda,      make_panda_chain,      make_panda_kdl_chain,      make_panda_kdl_limits,      7);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(panda,      make_panda_chain,      7, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(fetch,      make_fetch_chain,      make_fetch_kdl_chain,      make_fetch_kdl_limits,      7);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(fetch,      make_fetch_chain,      7, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(baxter,     make_baxter_chain,     make_baxter_kdl_chain,     make_baxter_kdl_limits,     7);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(baxter,     make_baxter_chain,     7, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);
IK_BENCH_ROBOT(kuka_lwr4,  make_kuka_lwr4_chain,  make_kuka_lwr4_kdl_chain,  make_kuka_lwr4_kdl_limits,  7);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, dls,            cartan::ik::dls,            cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, builtin_lm,     cartan::ik::builtin_lm,     cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, builtin_lbfgsb, cartan::ik::builtin_lbfgsb, cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, newton_raphson, cartan::ik::newton_raphson, cartan::ik::bench::newton_family_total_units);
IK_BENCH_DEFAULT_ONLY(kuka_lwr4,  make_kuka_lwr4_chain,  7, projected_lm,   cartan::ik::projected_lm,   cartan::ik::bench::projected_lm_total_units);

#ifdef CARTAN_BUILD_ARGMIN
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, argmin_lm,                    cartan::ik::argmin_lm,                    cartan::ik::bench::lm_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, argmin_slsqp,                 cartan::ik::argmin_slsqp,                 cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, argmin_bobyqa,                cartan::ik::argmin_bobyqa,                cartan::ik::bench::bobyqa_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, argmin_lbfgsb,                cartan::ik::argmin_lbfgsb,                cartan::ik::bench::lbfgsb_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, argmin_projected_gn,          cartan::ik::argmin_projected_gn,          cartan::ik::bench::projected_gn_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, argmin_projected_gradient_gn, cartan::ik::argmin_projected_gradient_gn, cartan::ik::bench::projected_gn_family_total_units);

// Fast-mode SQP cells — argmin sqp_mode::fast NTTP threaded through kraft_slsqp_policy.
IK_BENCH_SOLVER_VARIANTS(ur3e,       make_ur3e_chain,       6, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kr6_sixx,   make_kr6_sixx_chain,   6, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(abb_irb120, make_abb_irb120_chain, 6, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(jaco2,      make_jaco2_chain,      6, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(lbr_med14,  make_lbr_med14_chain,  7, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(panda,      make_panda_chain,      7, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(fetch,      make_fetch_chain,      7, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(baxter,     make_baxter_chain,     7, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);
IK_BENCH_SOLVER_VARIANTS(kuka_lwr4,  make_kuka_lwr4_chain,  7, argmin_slsqp_fast,            cartan::ik::argmin_slsqp_fast,            cartan::ik::bench::sqp_family_total_units);

// multiplier_reest_every_k sweep cells — fast-mode argmin_slsqp at K ∈ {1, 2, 5, 10}.
// 9 robots × 3 variants × 4 K values = 108 cells. Filter regex: `kreest_k`.
// Cell semantics: identical to the argmin_slsqp_fast cells above, but with
// options.multiplier_reest_every_k explicitly set per cell (overriding the
// per-Mode default of 5 on sqp_mode::fast). Designed to empirically verify
// argmin's "kraft KKT-leg is a behavioral no-op across k" prediction on the
// cartan IK pose-batch corpus.
#define CARTAN_KREEST_SWEEP_ROW(ROBOT, FACTORY, N_DOF, K_VALUE)                              \
    IK_BENCH_ARGMIN_SLSQP_KREEST_VARIANTS(ROBOT, FACTORY, N_DOF, argmin_slsqp_fast,          \
        cartan::ik::argmin_slsqp_fast, cartan::ik::bench::sqp_family_total_units, K_VALUE)

CARTAN_KREEST_SWEEP_ROW(ur3e,       make_ur3e_chain,       6, 1);
CARTAN_KREEST_SWEEP_ROW(ur3e,       make_ur3e_chain,       6, 2);
CARTAN_KREEST_SWEEP_ROW(ur3e,       make_ur3e_chain,       6, 5);
CARTAN_KREEST_SWEEP_ROW(ur3e,       make_ur3e_chain,       6, 10);
CARTAN_KREEST_SWEEP_ROW(kr6_sixx,   make_kr6_sixx_chain,   6, 1);
CARTAN_KREEST_SWEEP_ROW(kr6_sixx,   make_kr6_sixx_chain,   6, 2);
CARTAN_KREEST_SWEEP_ROW(kr6_sixx,   make_kr6_sixx_chain,   6, 5);
CARTAN_KREEST_SWEEP_ROW(kr6_sixx,   make_kr6_sixx_chain,   6, 10);
CARTAN_KREEST_SWEEP_ROW(abb_irb120, make_abb_irb120_chain, 6, 1);
CARTAN_KREEST_SWEEP_ROW(abb_irb120, make_abb_irb120_chain, 6, 2);
CARTAN_KREEST_SWEEP_ROW(abb_irb120, make_abb_irb120_chain, 6, 5);
CARTAN_KREEST_SWEEP_ROW(abb_irb120, make_abb_irb120_chain, 6, 10);
CARTAN_KREEST_SWEEP_ROW(jaco2,      make_jaco2_chain,      6, 1);
CARTAN_KREEST_SWEEP_ROW(jaco2,      make_jaco2_chain,      6, 2);
CARTAN_KREEST_SWEEP_ROW(jaco2,      make_jaco2_chain,      6, 5);
CARTAN_KREEST_SWEEP_ROW(jaco2,      make_jaco2_chain,      6, 10);
CARTAN_KREEST_SWEEP_ROW(lbr_med14,  make_lbr_med14_chain,  7, 1);
CARTAN_KREEST_SWEEP_ROW(lbr_med14,  make_lbr_med14_chain,  7, 2);
CARTAN_KREEST_SWEEP_ROW(lbr_med14,  make_lbr_med14_chain,  7, 5);
CARTAN_KREEST_SWEEP_ROW(lbr_med14,  make_lbr_med14_chain,  7, 10);
CARTAN_KREEST_SWEEP_ROW(panda,      make_panda_chain,      7, 1);
CARTAN_KREEST_SWEEP_ROW(panda,      make_panda_chain,      7, 2);
CARTAN_KREEST_SWEEP_ROW(panda,      make_panda_chain,      7, 5);
CARTAN_KREEST_SWEEP_ROW(panda,      make_panda_chain,      7, 10);
CARTAN_KREEST_SWEEP_ROW(fetch,      make_fetch_chain,      7, 1);
CARTAN_KREEST_SWEEP_ROW(fetch,      make_fetch_chain,      7, 2);
CARTAN_KREEST_SWEEP_ROW(fetch,      make_fetch_chain,      7, 5);
CARTAN_KREEST_SWEEP_ROW(fetch,      make_fetch_chain,      7, 10);
CARTAN_KREEST_SWEEP_ROW(baxter,     make_baxter_chain,     7, 1);
CARTAN_KREEST_SWEEP_ROW(baxter,     make_baxter_chain,     7, 2);
CARTAN_KREEST_SWEEP_ROW(baxter,     make_baxter_chain,     7, 5);
CARTAN_KREEST_SWEEP_ROW(baxter,     make_baxter_chain,     7, 10);
CARTAN_KREEST_SWEEP_ROW(kuka_lwr4,  make_kuka_lwr4_chain,  7, 1);
CARTAN_KREEST_SWEEP_ROW(kuka_lwr4,  make_kuka_lwr4_chain,  7, 2);
CARTAN_KREEST_SWEEP_ROW(kuka_lwr4,  make_kuka_lwr4_chain,  7, 5);
CARTAN_KREEST_SWEEP_ROW(kuka_lwr4,  make_kuka_lwr4_chain,  7, 10);

// Spot-check accurate-mode at k=1 on a single robot (UR3e) — confirms the
// pre-Phase-41 baseline path on the wrapper's other Mode. Not part of the
// k-sweep aggregate; reported in the cross-repo reply as a sanity probe.
IK_BENCH_ARGMIN_SLSQP_KREEST_VARIANTS(ur3e, make_ur3e_chain, 6, argmin_slsqp,
    cartan::ik::argmin_slsqp, cartan::ik::bench::sqp_family_total_units, 1);
#endif


}
