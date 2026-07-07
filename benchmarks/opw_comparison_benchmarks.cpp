/// @file opw_comparison_benchmarks.cpp
/// @brief Branch-for-branch cross-check of cartan's opw_6r_solver against the
///        independent opw_kinematics reference (ROS-Industrial, Apache-2.0) on
///        the KUKA KR6 R900 SIXX fixture, plus head-to-head solve timing.
///
/// The same hand-derived KR6 R900 opw_parameters feed BOTH solvers (identical
/// struct layout; only the branch-column indexing differs). For every reachable
/// target each side's solutions are classified by cartan::classify_branch on
/// their joint values -- so the parity never trusts either library's emission
/// order -- and the two solutions of the SAME canonical branch are asserted to
/// agree joint-for-joint modulo 2*pi. This catches sign / offset / frame-
/// convention discrepancies a single-implementation check cannot.
///
/// As a secondary report, each valid reference column's classified branch is
/// checked against the documented canonical permutation
/// (cartan_key -> reference_col = {0->0,1->4,2->1,3->5,4->2,5->6,6->3,7->7},
/// equivalently reference_col -> cartan_key = {0,2,4,6,1,3,5,7}), decoded from
/// opw_kinematics_impl.h at the pinned SHA. A violation means the documented
/// column table drifted, not that cartan is wrong -- the joint-value match is
/// the correctness ground truth.
///
/// In the wrist-singular band (2.5e-8 < |sin theta5| < 1e-6) cartan uses its
/// empirically-swept fold threshold while the reference pins theta4 = 0 at a
/// hardcoded 1e-6, so the two emit different-but-both-FK-valid theta4/theta6
/// splits there. Those are counted separately as a documented divergence, not a
/// mismatch.
///
/// opw_kinematics: https://github.com/Jmeyer1292/opw_kinematics @ v0.5.5
/// (SHA 8a32bda8197c50bd0d60dfe1d12ecb4c13111b72), Apache-2.0. Header-only
/// reference target, linked ONLY into this benchmark executable.

#include "../tests/fixtures/opw_chains.h"

#include <cartan/analytical.h>
#include <cartan/serial_chain.h>

#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_kinematics_impl.h>

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

namespace
{

using kr6_chain = decltype(cartan::fixtures::make_kr6_r900_opw_chain<double>());
using kr6_solver = cartan::opw_6r_solver<kr6_chain>;
using kr6_solver_raw = cartan::opw_6r_solver<kr6_chain, cartan::opw_raw>;

constexpr int num_targets = 2000;

/// FK round-trip gate below which a reference column is accepted as a real IK
/// solution of the target (the reference marks unreachable branches with NaN,
/// but this also rejects any finite-but-spurious column).
constexpr double verify_tol = 1e-6;

/// Per-joint agreement bound (radians, modulo 2*pi) for two solutions of the
/// same canonical branch away from the wrist-singular band.
constexpr double match_tol = 1e-6;

/// |sin(theta5_internal)| below which theta4/theta6 are ill-conditioned and the
/// two solvers legitimately split differently; disagreement inside this band is
/// reported as a documented divergence rather than a mismatch.
constexpr double singular_band = 1e-6;

/// Documented reference_col -> cartan canonical key, decoded from the theta
/// packing matrix in opw_kinematics_impl.h at the pinned SHA.
constexpr std::array<int, 8> reference_col_to_cartan_key{0, 2, 4, 6, 1, 3, 5, 7};

opw_kinematics::Parameters<double> to_reference_params(
    const cartan::opw_parameters<double>& p)
{
    opw_kinematics::Parameters<double> r;
    r.a1 = p.a1;
    r.a2 = p.a2;
    r.b = p.b;
    r.c1 = p.c1;
    r.c2 = p.c2;
    r.c3 = p.c3;
    r.c4 = p.c4;
    r.offsets = p.offsets;
    r.sign_corrections = p.sign_corrections;
    return r;
}

opw_kinematics::Transform<double> se3_to_isometry(const cartan::se3<double>& pose)
{
    opw_kinematics::Transform<double> iso = opw_kinematics::Transform<double>::Identity();
    iso.linear() = pose.rotation().matrix();
    iso.translation() = pose.translation();
    return iso;
}

/// Smallest signed representative of a - b on (-pi, pi].
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

/// |sin(theta5_internal)| for the wrist-singular-band guard.
double abs_sin_theta5(const Eigen::Vector<double, 6>& q,
                      const cartan::opw_parameters<double>& p)
{
    const double t5 = q(4) * static_cast<double>(p.sign_corrections[4])
        - p.offsets[4];
    return std::abs(std::sin(t5));
}

struct opw_fixture
{
    kr6_chain chain;
    cartan::opw_parameters<double> params;
    kr6_solver solver;
    kr6_solver_raw solver_raw;
    opw_kinematics::Parameters<double> ref_params;
    std::vector<cartan::se3<double>> targets;
    std::vector<opw_kinematics::Transform<double>> iso_targets;
};

opw_fixture build_fixture()
{
    auto chain = cartan::fixtures::make_kr6_r900_opw_chain<double>();
    auto params = cartan::fixtures::kr6_r900_opw_parameters<double>();
    auto maybe = kr6_solver::make(chain, params);
    auto maybe_raw = kr6_solver_raw::make(chain, params);
    if (!maybe || !maybe_raw)
    {
        std::cerr << "opw_comparison_benchmarks: could not construct opw_6r_solver "
                     "for the KR6 R900 fixture\n";
        std::abort();
    }

    std::vector<cartan::se3<double>> targets;
    std::vector<opw_kinematics::Transform<double>> iso_targets;
    targets.reserve(num_targets);
    iso_targets.reserve(num_targets);

    // FK-walk random configurations so every target is reachable by construction.
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-3.0, 3.0);
    for (int i = 0; i < num_targets; ++i)
    {
        Eigen::Vector<double, 6> q;
        for (int j = 0; j < 6; ++j)
            q(j) = dist(rng);
        auto pose = cartan::forward_kinematics(chain, q).end_effector;
        targets.push_back(pose);
        iso_targets.push_back(se3_to_isometry(pose));
    }

    return opw_fixture{
        chain,
        params,
        std::move(*maybe),
        std::move(*maybe_raw),
        to_reference_params(params),
        std::move(targets),
        std::move(iso_targets)};
}

const opw_fixture& fixture()
{
    static const opw_fixture fx = build_fixture();
    return fx;
}

struct parity_stats
{
    long targets_solved = 0;
    long targets_skipped = 0; // cartan reported unreachable/singular
    long cartan_branches = 0;
    long reference_valid_cols = 0;
    long exact_matches = 0;
    long singular_band_splits = 0;
    long hard_mismatches = 0;
    long cartan_only = 0;
    long reference_only = 0;
    long column_table_violations = 0;
    double max_disagreement = 0.0;
    Eigen::Vector<double, 6> worst_cartan = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 6> worst_reference = Eigen::Vector<double, 6>::Zero();
};

parity_stats run_parity(const opw_fixture& fx)
{
    parity_stats st;

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

        // cartan side: canonical key -> joint solution (FK-verified internally).
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

        // reference side: filter to columns that reconstruct the target, then
        // classify by cartan's own predicate.
        const auto ref = opw_kinematics::inverse(fx.ref_params, fx.iso_targets[t]);
        std::array<bool, 8> ref_present{};
        std::array<Eigen::Vector<double, 6>, 8> ref_q{};
        for (int col = 0; col < 8; ++col)
        {
            Eigen::Vector<double, 6> q;
            bool finite = true;
            for (int j = 0; j < 6; ++j)
            {
                q(j) = ref[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)];
                finite = finite && std::isfinite(q(j));
            }
            if (!finite)
                continue;

            const auto fk = cartan::forward_kinematics(fx.chain, q).end_effector;
            const double pe = (fk.translation() - target.translation()).norm();
            const double oe =
                (fk.rotation().inverse() * target.rotation()).log().norm();
            if (std::max(pe, oe) > verify_tol)
                continue;

            const int key = static_cast<int>(cartan::classify_branch(q, fx.params));
            ref_present[static_cast<std::size_t>(key)] = true;
            ref_q[static_cast<std::size_t>(key)] = q;
            ++st.reference_valid_cols;

            if (reference_col_to_cartan_key[static_cast<std::size_t>(col)] != key)
                ++st.column_table_violations;
        }

        // branch-for-branch parity.
        for (std::size_t key = 0; key < 8; ++key)
        {
            const bool c = cart_present[key];
            const bool r = ref_present[key];
            if (c && r)
            {
                const double d = max_wrapped_diff(cart_q[key], ref_q[key]);
                if (d > st.max_disagreement)
                {
                    st.max_disagreement = d;
                    st.worst_cartan = cart_q[key];
                    st.worst_reference = ref_q[key];
                }
                if (d <= match_tol)
                    ++st.exact_matches;
                else if (abs_sin_theta5(cart_q[key], fx.params) < singular_band)
                    ++st.singular_band_splits;
                else
                    ++st.hard_mismatches;
            }
            else if (c && !r)
            {
                ++st.cartan_only;
            }
            else if (!c && r)
            {
                ++st.reference_only;
            }
        }
    }

    return st;
}

bool print_receipt(const parity_stats& st)
{
    const bool pass = st.hard_mismatches == 0 && st.column_table_violations == 0
        && st.cartan_only == 0 && st.reference_only == 0;

    std::cout
        << "\n=== OPW branch-for-branch parity: cartan opw_6r_solver vs "
           "opw_kinematics (KR6 R900) ===\n"
        << "targets solved (cartan)      : " << st.targets_solved << "\n"
        << "targets skipped (unreachable): " << st.targets_skipped << "\n"
        << "cartan branches (total)      : " << st.cartan_branches << "\n"
        << "reference valid columns      : " << st.reference_valid_cols << "\n"
        << "branches matched exact       : " << st.exact_matches << "\n"
        << "  (|dq| <= " << match_tol << " mod 2*pi)\n"
        << "singular-band th4/th6 splits : " << st.singular_band_splits << "\n"
        << "cartan-only branches         : " << st.cartan_only << "\n"
        << "reference-only branches      : " << st.reference_only << "\n"
        << "documented column violations : " << st.column_table_violations << "\n"
        << "max joint disagreement (rad) : "
        << std::setprecision(17) << st.max_disagreement << std::setprecision(6) << "\n";

    if (st.max_disagreement > 0.0)
    {
        std::cout << std::setprecision(17)
                  << "  worst cartan    : " << st.worst_cartan.transpose() << "\n"
                  << "  worst reference : " << st.worst_reference.transpose() << "\n"
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

void bm_opw_kinematics_inverse(benchmark::State& state)
{
    const auto& fx = fixture();
    std::size_t i = 0;
    for (auto _ : state)
    {
        auto s = opw_kinematics::inverse(
            fx.ref_params, fx.iso_targets[i % fx.iso_targets.size()]);
        benchmark::DoNotOptimize(s);
        ++i;
    }
}
BENCHMARK(bm_opw_kinematics_inverse);

} // namespace

int main(int argc, char** argv)
{
    const bool pass = print_receipt(run_parity(fixture()));

    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();

    return pass ? 0 : 1;
}
