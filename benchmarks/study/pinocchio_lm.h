#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_LM_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_LM_H

/// @file pinocchio_lm.h
/// @brief The peer library's Levenberg-Marquardt, driven by this repository.
///
/// Body-frame error, body Jacobian, Nielsen damping, LDLT normal-equation
/// solve: the same design cartan's own policy implements. The loop is written
/// here, which is why its kernel evaluations are counted at the call sites
/// rather than inferred from an iteration count.

#include "solve_outcome.h"
#include "counting_chain.h"
#include "pinocchio_chain.h"

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>

namespace cartan::bench::detail
{

struct pin_lm_scratch
{
    Eigen::MatrixXd jac;
    Eigen::MatrixXd normal;
    Eigen::VectorXd gradient;
    Eigen::VectorXd step;
    Eigen::VectorXd trial;

    explicit pin_lm_scratch(int n)
        : jac(Eigen::MatrixXd::Zero(6, n))
        , normal(n, n)
        , gradient(n)
        , step(n)
        , trial(n)
    {
    }
};

struct pin_lm_state
{
    Eigen::VectorXd q;
    Eigen::Matrix<double, 6, 1> error;
    double squared;
    double lambda;
    double nu;
};

struct pin_lm_answer
{
    Eigen::VectorXd q;
    bool converged;
    int iterations;
};

inline Eigen::Matrix<double, 6, 1> pin_error(
    pinocchio_model& peer, const Eigen::VectorXd& q, const pinocchio::SE3& goal,
    kernel_counts& counts)
{
    ++counts.fk;
    pinocchio::framesForwardKinematics(peer.model, peer.data, q);
    return pinocchio::log6(peer.data.oMf[peer.ee_frame].actInv(goal)).toVector();
}

inline void pin_jacobian(
    pinocchio_model& peer, const Eigen::VectorXd& q, Eigen::MatrixXd& jac, kernel_counts& counts)
{
    ++counts.jac;
    pinocchio::computeFrameJacobian(peer.model, peer.data, q, peer.ee_frame, pinocchio::LOCAL, jac);
}

/// pinocchio's log6 lays a motion out as linear then angular, the reverse of
/// the omega-first convention the rest of this study reads.
inline bool pin_converged(const Eigen::Matrix<double, 6, 1>& error, double tolerance)
{
    return error.head<3>().norm() < tolerance && error.tail<3>().norm() < tolerance;
}

inline double pin_initial_damping(const Eigen::MatrixXd& normal)
{
    const double lambda = 1e-3 * normal.diagonal().maxCoeff();
    return lambda < std::numeric_limits<double>::epsilon() ? 1e-4 : lambda;
}

inline void pin_propose_step(
    pinocchio_model& peer, pin_lm_scratch& work, const pin_lm_state& state, kernel_counts& counts)
{
    pin_jacobian(peer, state.q, work.jac, counts);
    work.normal.noalias() = work.jac.transpose() * work.jac;
    work.gradient.noalias() = work.jac.transpose() * state.error;
    work.normal.diagonal().array() += state.lambda;
    work.step = work.normal.ldlt().solve(work.gradient);
    work.trial = state.q + work.step;
}

inline double pin_gain_ratio(
    const pin_lm_scratch& work, const pin_lm_state& state, double trial_squared)
{
    const double predicted = work.step.dot(state.lambda * work.step + work.gradient);
    if (std::abs(predicted) <= std::numeric_limits<double>::epsilon())
    {
        return 0.0;
    }
    return (state.squared - trial_squared) / predicted;
}

inline void pin_lm_step(
    pinocchio_model& peer,
    pin_lm_scratch& work,
    const pinocchio::SE3& goal,
    pin_lm_state& state,
    kernel_counts& counts)
{
    pin_propose_step(peer, work, state, counts);
    const auto trial_error = pin_error(peer, work.trial, goal, counts);
    const double trial_squared = trial_error.squaredNorm();
    const double gain = pin_gain_ratio(work, state, trial_squared);
    if (gain <= 0.0)
    {
        state.lambda *= state.nu;
        state.nu *= 2.0;
        return;
    }
    state.q = work.trial;
    state.error = trial_error;
    state.squared = trial_squared;
    state.lambda *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * gain - 1.0, 3.0));
    state.nu = 2.0;
}

inline pin_lm_answer run_pinocchio_lm(
    pinocchio_model& peer,
    pin_lm_scratch& work,
    const pinocchio::SE3& goal,
    Eigen::VectorXd seed,
    const solve_budget& budget,
    kernel_counts& counts)
{
    const auto error = pin_error(peer, seed, goal, counts);
    pin_jacobian(peer, seed, work.jac, counts);
    work.normal.noalias() = work.jac.transpose() * work.jac;
    pin_lm_state state{std::move(seed), error, error.squaredNorm(),
        pin_initial_damping(work.normal), 2.0};

    for (int iteration = 0; iteration < budget.units; ++iteration)
    {
        if (pin_converged(state.error, budget.tolerance))
        {
            return pin_lm_answer{state.q, true, iteration};
        }
        pin_lm_step(peer, work, goal, state, counts);
    }
    return pin_lm_answer{state.q, pin_converged(state.error, budget.tolerance), budget.units};
}

}

#endif
