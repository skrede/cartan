#ifndef HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_ANALYTICAL_PYTHON_HELPERS_H
#define HPP_GUARD_CARTAN_PYTHON_BINDINGS_DETAIL_ANALYTICAL_PYTHON_HELPERS_H

/// Binding-internal value types and unwrap helpers for the closed-form
/// analytical IK surface. AnalyticalResult is the always-returned shape
/// carrying the multi-solution joint vectors, a coarse-grained status
/// enum (py_analytical_status), and an error_metric magnitude.
///
/// py_analytical_status mirrors cartan::analytical_failure plus a Python
/// success sentinel "ok". The C++ analytical_failure variants are
/// unreachable, degenerate_geometry, singular_configuration, and
/// verification_failed. The to_py_status helper translates a C++
/// failure value to the Python enum; the success branch is handled at
/// the caller, which constructs an AnalyticalResult with status=ok.
///
/// The to_analytical_result template unwraps a
/// cartan::expected<analytical_result, analytical_error> into the
/// Python value type. On success it iterates only the populated prefix
/// of the std::array<position_type, MaxSolutions> (count entries) and
/// emplaces fresh Eigen::VectorXd instances that nanobind marshals to
/// a Python list[ndarray]. On failure it captures the analytical_error
/// reason and workspace_distance.

#include "cartan/expected.h"
#include "cartan/analytical/range_status.h"
#include "cartan/analytical/analytical_types.h"

#include <Eigen/Core>

#include <vector>
#include <cstddef>
#include <utility>

namespace cartan::python {

enum class py_analytical_status : int
{
    ok,
    unreachable,
    degenerate_geometry,
    singular_configuration,
    verification_failed
};

inline py_analytical_status to_py_status(cartan::analytical_failure f)
{
    switch(f)
    {
        case cartan::analytical_failure::unreachable:
            return py_analytical_status::unreachable;
        case cartan::analytical_failure::degenerate_geometry:
            return py_analytical_status::degenerate_geometry;
        case cartan::analytical_failure::singular_configuration:
            return py_analytical_status::singular_configuration;
        case cartan::analytical_failure::verification_failed:
            return py_analytical_status::verification_failed;
    }
    return py_analytical_status::degenerate_geometry;
}

struct AnalyticalResult
{
    std::vector<Eigen::VectorXd> solutions;
    py_analytical_status status{py_analytical_status::ok};
    double error_metric{0.0};
};

struct UnwrappedResult
{
    std::vector<Eigen::VectorXd> solutions;
    std::vector<cartan::range_status> tags;
    py_analytical_status status{py_analytical_status::ok};
    double error_metric{0.0};
};

inline AnalyticalResult to_analytical_error_result(const cartan::analytical_error<double> &err)
{
    AnalyticalResult out;
    out.status       = to_py_status(err.reason);
    out.error_metric = err.workspace_distance;
    return out;
}

template<int N, int MaxSolutions>
inline AnalyticalResult to_analytical_result(cartan::expected<cartan::analytical_result<double, N, MaxSolutions>, cartan::analytical_error<double>> &&r)
{
    if(!r)
    {
        auto err = std::move(r).error();
        return to_analytical_error_result(err);
    }
    AnalyticalResult out;
    out.solutions.reserve(static_cast<std::size_t>(r->count));
    for(int i = 0; i < r->count; ++i)
    {
        out.solutions.emplace_back(r->solutions[static_cast<std::size_t>(i)]);
    }
    out.status       = py_analytical_status::ok;
    out.error_metric = 0.0;
    return out;
}

}

#endif
