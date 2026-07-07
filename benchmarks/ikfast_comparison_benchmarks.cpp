/// @file ikfast_comparison_benchmarks.cpp
/// @brief Independent-formulation cross-check of cartan's opw_6r_solver against
///        an OpenRAVE-generated IKFast solver for the KUKA KR6 R900 SIXX, plus
///        head-to-head solve timing.
///
/// IKFast derives the closed form per-robot by algebraic elimination from the
/// kinematics -- unlike opw_kinematics (which shares the Brandstotter OPW
/// parameterization and agrees with cartan to the bit), IKFast is a genuinely
/// independent derivation, so agreement is expected at ~machine precision, not
/// bit-identical. That independence is exactly its value as a witness: it can
/// catch an error common to any OPW-parameterized solver.
///
/// Frame reconciliation: the generated solver targets the link_6 (flange) frame,
/// identity-oriented at the zero configuration, whereas cartan's KR6 fixture
/// carries a home rotation R_home = Ry(90 deg). All six joint axes and the joint
/// zeros match cartan's screw model (both derive from the same URDF), so the ONLY
/// reconciliation is a right-multiply on the query rotation: feed IKFast
/// eerot = R_target * R_home^{-1} (position unchanged). The recovered joint
/// vectors are then directly comparable to cartan's -- no per-joint sign/offset
/// remapping -- and each is FK-verified against the target with cartan's own
/// forward kinematics before being classified by cartan::classify_branch.
///
/// The generated solver is vendored under third_party/ikfast_kr6r900/ (see its
/// README for provenance); it links only into this benchmark.

#include "../tests/fixtures/opw_chains.h"

#include <cartan/analytical.h>
#include <cartan/serial_chain.h>

#define IKFAST_HAS_LIBRARY
#include <ikfast.h>

#include <Eigen/Geometry>

#include <benchmark/benchmark.h>

#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <array>
#include <vector>
#include <random>
#include <numbers>
#include <iomanip>
#include <iostream>

// ComputeIk / ComputeFk are declared by ikfast.h (above) and defined in the
// vendored generated solver; both build with C++ linkage.

namespace
{

using kr6_chain = decltype(cartan::fixtures::make_kr6_r900_opw_chain<double>());
using kr6_solver = cartan::opw_6r_solver<kr6_chain>;
using kr6_solver_raw = cartan::opw_6r_solver<kr6_chain, cartan::opw_raw>;

constexpr int num_targets = 2000;

/// FK round-trip gate below which an IKFast solution is accepted as a real IK
/// solution of the target.
constexpr double verify_tol = 1e-6;

/// Per-joint agreement bound (radians, modulo 2*pi) for two solutions of the
/// same canonical branch. Loose enough for an independent formulation's ~1e-9
/// numerical divergence, tight enough to catch a real discrepancy.
constexpr double match_tol = 1e-6;

double wrap_to_pi(double angle)
{
    constexpr double two_pi = 2.0 * std::numbers::pi;
    double a = std::fmod(angle + std::numbers::pi, two_pi);
    if (a <= 0.0)
        a += two_pi;
    return a - std::numbers::pi;
}

double max_wrapped_diff(const Eigen::Vector<double, 6>& a,
                        const Eigen::Vector<double, 6>& b)
{
    double m = 0.0;
    for (int i = 0; i < 6; ++i)
        m = std::max(m, std::abs(wrap_to_pi(a(i) - b(i))));
    return m;
}

struct ikfast_fixture
{
    kr6_chain chain;
    cartan::opw_parameters<double> params;
    kr6_solver solver;
    kr6_solver_raw solver_raw;
    Eigen::Matrix3d r_home_inv;
    std::vector<cartan::se3<double>> targets;
    std::vector<std::array<double, 9>> eerot; // row-major R_target * R_home^{-1}
    std::vector<std::array<double, 3>> eetrans;
};

ikfast_fixture build_fixture()
{
    auto chain = cartan::fixtures::make_kr6_r900_opw_chain<double>();
    auto params = cartan::fixtures::kr6_r900_opw_parameters<double>();
    auto maybe = kr6_solver::make(chain, params);
    auto maybe_raw = kr6_solver_raw::make(chain, params);
    if (!maybe || !maybe_raw)
    {
        std::cerr << "ikfast_comparison_benchmarks: could not construct "
                     "opw_6r_solver for the KR6 R900 fixture\n";
        std::abort();
    }

    const Eigen::Matrix3d r_home =
        cartan::forward_kinematics(chain, Eigen::Vector<double, 6>::Zero())
            .end_effector.rotation()
            .matrix();
    const Eigen::Matrix3d r_home_inv = r_home.transpose();

    std::vector<cartan::se3<double>> targets;
    std::vector<std::array<double, 9>> eerot;
    std::vector<std::array<double, 3>> eetrans;
    targets.reserve(num_targets);
    eerot.reserve(num_targets);
    eetrans.reserve(num_targets);

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-3.0, 3.0);
    for (int i = 0; i < num_targets; ++i)
    {
        Eigen::Vector<double, 6> q;
        for (int j = 0; j < 6; ++j)
            q(j) = dist(rng);
        auto pose = cartan::forward_kinematics(chain, q).end_effector;

        const Eigen::Matrix3d rq = pose.rotation().matrix() * r_home_inv;
        std::array<double, 9> er{};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                er[static_cast<std::size_t>(3 * r + c)] = rq(r, c); // row-major
        std::array<double, 3> et{
            pose.translation()(0), pose.translation()(1), pose.translation()(2)};

        targets.push_back(pose);
        eerot.push_back(er);
        eetrans.push_back(et);
    }

    return ikfast_fixture{
        chain,
        params,
        std::move(*maybe),
        std::move(*maybe_raw),
        r_home_inv,
        std::move(targets),
        std::move(eerot),
        std::move(eetrans)};
}

const ikfast_fixture& fixture()
{
    static const ikfast_fixture fx = build_fixture();
    return fx;
}

struct parity_stats
{
    long targets_solved = 0;
    long targets_skipped = 0;
    long cartan_branches = 0;
    long ikfast_exact_solutions = 0; // FK-verified against the target
    long exact_matches = 0;
    long hard_mismatches = 0;
    long cartan_only = 0;
    long ikfast_only = 0;
    double max_disagreement = 0.0;
    Eigen::Vector<double, 6> worst_cartan = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 6> worst_ikfast = Eigen::Vector<double, 6>::Zero();
};

parity_stats run_parity(const ikfast_fixture& fx)
{
    parity_stats st;
    std::vector<double> qbuf(6);
    std::vector<double> nofree;

    for (std::size_t t = 0; t < fx.targets.size(); ++t)
    {
        const auto& target = fx.targets[t];
        auto cartan_result = fx.solver.solve(target);
        if (!cartan_result)
        {
            ++st.targets_skipped;
            continue;
        }
        ++st.targets_solved;

        std::array<bool, 8> cart_present{};
        std::array<Eigen::Vector<double, 6>, 8> cart_q{};
        for (int i = 0; i < cartan_result->count; ++i)
        {
            const auto& q = cartan_result->solutions[static_cast<std::size_t>(i)];
            const int key = static_cast<int>(cartan::classify_branch(q, fx.params));
            cart_present[static_cast<std::size_t>(key)] = true;
            cart_q[static_cast<std::size_t>(key)] = q;
            ++st.cartan_branches;
        }

        ikfast::IkSolutionList<double> sols;
        ComputeIk(fx.eetrans[t].data(), fx.eerot[t].data(), nullptr, sols);
        const std::size_t nsol = sols.GetNumSolutions();

        std::array<bool, 8> if_present{};
        std::array<Eigen::Vector<double, 6>, 8> if_q{};
        for (std::size_t k = 0; k < nsol; ++k)
        {
            sols.GetSolution(k).GetSolution(qbuf, nofree);
            Eigen::Vector<double, 6> q;
            for (int j = 0; j < 6; ++j)
                q(j) = qbuf[static_cast<std::size_t>(j)];

            const auto fk = cartan::forward_kinematics(fx.chain, q).end_effector;
            const double pe = (fk.translation() - target.translation()).norm();
            const double oe =
                (fk.rotation().inverse() * target.rotation()).log().norm();
            if (std::max(pe, oe) > verify_tol)
                continue;

            const int key = static_cast<int>(cartan::classify_branch(q, fx.params));
            if_present[static_cast<std::size_t>(key)] = true;
            if_q[static_cast<std::size_t>(key)] = q;
            ++st.ikfast_exact_solutions;
        }

        for (std::size_t key = 0; key < 8; ++key)
        {
            const bool c = cart_present[key];
            const bool r = if_present[key];
            if (c && r)
            {
                const double d = max_wrapped_diff(cart_q[key], if_q[key]);
                if (d > st.max_disagreement)
                {
                    st.max_disagreement = d;
                    st.worst_cartan = cart_q[key];
                    st.worst_ikfast = if_q[key];
                }
                if (d <= match_tol)
                    ++st.exact_matches;
                else
                    ++st.hard_mismatches;
            }
            else if (c && !r)
            {
                ++st.cartan_only;
            }
            else if (!c && r)
            {
                ++st.ikfast_only;
            }
        }
    }

    return st;
}

bool print_receipt(const parity_stats& st)
{
    const bool pass = st.hard_mismatches == 0 && st.cartan_only == 0
        && st.ikfast_only == 0 && st.exact_matches > 0;

    std::cout
        << "\n=== Branch-for-branch parity: cartan opw_6r_solver vs IKFast "
           "(KR6 R900, Transform6D) ===\n"
        << "targets solved (cartan)      : " << st.targets_solved << "\n"
        << "targets skipped (unreachable): " << st.targets_skipped << "\n"
        << "cartan branches (total)      : " << st.cartan_branches << "\n"
        << "ikfast exact solutions       : " << st.ikfast_exact_solutions << "\n"
        << "branches matched             : " << st.exact_matches << "\n"
        << "  (|dq| <= " << match_tol << " mod 2*pi)\n"
        << "hard mismatches              : " << st.hard_mismatches << "\n"
        << "cartan-only branches         : " << st.cartan_only << "\n"
        << "ikfast-only branches         : " << st.ikfast_only << "\n"
        << "max joint disagreement (rad) : "
        << std::setprecision(17) << st.max_disagreement << std::setprecision(6)
        << "\n";

    if (st.max_disagreement > match_tol)
    {
        std::cout << std::setprecision(17)
                  << "  worst cartan : " << st.worst_cartan.transpose() << "\n"
                  << "  worst ikfast : " << st.worst_ikfast.transpose() << "\n"
                  << std::setprecision(6);
    }

    std::cout << "RESULT: " << (pass ? "PASS" : "FAIL") << "\n\n";
    return pass;
}

void bm_cartan_opw_6r_solve(benchmark::State& state)
{
    const auto& fx = fixture();
    std::size_t i = 0;
    for (auto _ : state)
    {
        auto r = fx.solver.solve(fx.targets[i % fx.targets.size()]);
        benchmark::DoNotOptimize(r);
        ++i;
    }
}
BENCHMARK(bm_cartan_opw_6r_solve);

void bm_cartan_opw_6r_solve_raw(benchmark::State& state)
{
    const auto& fx = fixture();
    std::size_t i = 0;
    for (auto _ : state)
    {
        auto r = fx.solver_raw.solve(fx.targets[i % fx.targets.size()]);
        benchmark::DoNotOptimize(r);
        ++i;
    }
}
BENCHMARK(bm_cartan_opw_6r_solve_raw);

void bm_ikfast_compute_ik(benchmark::State& state)
{
    const auto& fx = fixture();
    std::size_t i = 0;
    for (auto _ : state)
    {
        ikfast::IkSolutionList<double> sols;
        const std::size_t t = i % fx.targets.size();
        ComputeIk(fx.eetrans[t].data(), fx.eerot[t].data(), nullptr, sols);
        benchmark::DoNotOptimize(sols.GetNumSolutions());
        ++i;
    }
}
BENCHMARK(bm_ikfast_compute_ik);

} // namespace

int main(int argc, char** argv)
{
    const bool pass = print_receipt(run_parity(fixture()));

    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();

    return pass ? 0 : 1;
}
