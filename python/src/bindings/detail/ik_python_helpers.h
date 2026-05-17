#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_IK_PYTHON_HELPERS_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_IK_PYTHON_HELPERS_H

/// Binding-internal value types and per-site lambdas for the iterative IK
/// surface. IkResult is the always-returned shape carrying the diagnostic
/// fields requested by the Python contract; IkConfig is the keyword-only
/// configuration value the binding lambdas unpack into the C++
/// convergence_criteria + solver_options pair before driving a runner.
///
/// The to_ik_result / to_ik_result_from_error helpers unwrap a
/// cartan::expected<ik_result, ik_error> into IkResult. The success branch
/// leaves condition_number = 0.0 because the runner does not compute the
/// Jacobian SVD on convergence (a Jacobian condition number is only
/// populated on the failure path).

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/ik_result.h"

#include <Eigen/Core>

#include <string>
#include <utility>

namespace cartan::python
{

struct IkResult
{
    Eigen::VectorXd q;
    bool converged{false};
    int iterations{0};
    double error_norm{0.0};
    std::string failure_reason;
    int solver_index{0};
    cartan::ik_termination_reason termination_reason{cartan::ik_termination_reason::unknown};
    bool near_singular{false};
    double condition_number{0.0};
};

struct IkConfig
{
    int max_iterations_per_attempt{100};
    int max_total_work_units{200};
    double position_tol{1e-6};
    double orientation_tol{1e-6};
    int max_total_iterations{500};
    cartan::ik_objective objective{cartan::ik_objective::speed};
    unsigned int halton_seed{42};
};

inline std::string ik_failure_to_string(cartan::ik_failure r)
{
    switch (r)
    {
        case cartan::ik_failure::unreachable:           return "unreachable";
        case cartan::ik_failure::diverged:              return "diverged";
        case cartan::ik_failure::stalled:               return "stalled";
        case cartan::ik_failure::iteration_limit:       return "iteration_limit";
        case cartan::ik_failure::joint_limit_violation: return "joint_limit_violation";
        case cartan::ik_failure::aborted:               return "aborted";
    }
    return "unknown";
}

template <int N>
inline IkResult to_ik_result(cartan::ik_result<double, N>&& ok)
{
    IkResult out;
    out.q                  = std::move(ok.solution.position);
    out.converged          = true;
    out.iterations         = ok.iterations;
    out.error_norm         = ok.final_error_norm;
    out.failure_reason     = "";
    out.solver_index       = ok.solver_index;
    out.termination_reason = cartan::ik_termination_reason::converged;
    out.near_singular      = false;
    out.condition_number   = 0.0;
    return out;
}

template <int N>
inline IkResult to_ik_result_from_error(cartan::ik_error<double, N>&& err, int iters)
{
    IkResult out;
    out.q                  = std::move(err.last_q);
    out.converged          = false;
    out.iterations         = iters;
    out.error_norm         = err.last_error_norm;
    out.failure_reason     = ik_failure_to_string(err.reason);
    out.solver_index       = -1;
    out.termination_reason = err.termination_reason;
    out.near_singular      = err.near_singular;
    out.condition_number   = err.condition_number;
    return out;
}

}

#endif
