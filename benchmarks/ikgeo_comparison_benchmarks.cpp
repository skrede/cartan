/// @file ikgeo_comparison_benchmarks.cpp
/// @brief Independent-formulation cross-check of cartan's opw_6r_solver against
///        ik-geo (Elias & Wen, RPI) on the KUKA KR6 R900 SIXX, plus solve timing.
///
/// ik-geo derives the inverse by canonical geometric subproblems -- an
/// independent formulation from the OPW parameterization opw_kinematics shares
/// with cartan -- so agreement is expected at ~machine precision, not
/// bit-identical. It is reached through a small C FFI over ik-geo's Rust crate
/// (its C++ port's spherical solver is broken for roll-pitch-roll wrists; the
/// Rust one is not).
///
/// The two solvers are fed the SAME physical KR6, derived programmatically from
/// cartan's screw model: ik-geo H.col(i) = joint axis omega_i, and its P columns
/// are the inter-axis link vectors from each axis's point g_i = omega_i x v_i,
/// with the three spherical-wrist axes collapsed onto their least-squares common
/// intersection (ik-geo's FK is invariant to per-axis point choice, but its
/// solver needs p45 = p56 = 0). ik-geo hardcodes its tool offset R_6T = I, so the
/// reconciliation is a single right-multiply on the query rotation: feed ik-geo
/// R_0T = R_target * R_home^{-1} (position unchanged). Each ik-geo solution is
/// filtered for the least-squares flag, FK-verified against the target with
/// cartan's own forward kinematics, then classified by cartan::classify_branch so
/// the branch-for-branch match never trusts either library's ordering.
///
/// ik-geo: https://github.com/rpiRobotics/ik-geo @ a3a1675 (BSD-3-Clause), driven
/// via the ikgeo_ffi shim (linked only into this benchmark).

#include "../tests/fixtures/opw_chains.h"

#include <cartan/analytical.h>
#include <cartan/serial_chain.h>

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

// ik-geo Rust crate via the ikgeo_ffi shim. h: 18 doubles column-major (3x6);
// p: 21 doubles column-major (3x7); r: 9 doubles row-major (3x3); t: 3 doubles.
// out_q >= 48, out_is_ls >= 8. Returns solution count (<= 8).
extern "C" unsigned long ikgeo_spherical_two_parallel(
    const double* h, const double* p, const double* r, const double* t,
    double* out_q, unsigned char* out_is_ls);

namespace
{

using kr6_chain = decltype(cartan::fixtures::make_kr6_r900_opw_chain<double>());
using kr6_solver = cartan::opw_6r_solver<kr6_chain>;
using kr6_solver_raw = cartan::opw_6r_solver<kr6_chain, cartan::opw_raw>;

constexpr int num_targets = 2000;
constexpr double verify_tol = 1e-6;
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

/// Column-major flat H (3x6) and P (3x7) for ik-geo, from cartan's screw model.
void make_ikgeo_hp(const kr6_chain& chain,
                   std::array<double, 18>& h_flat,
                   std::array<double, 21>& p_flat)
{
    Eigen::Matrix<double, 3, 6> hm;
    std::array<Eigen::Vector3d, 6> g;
    for (int i = 0; i < 6; ++i)
    {
        const Eigen::Vector3d w = chain.axis(i).omega();
        const Eigen::Vector3d v = chain.axis(i).v();
        hm.col(i) = w;
        g[static_cast<std::size_t>(i)] = w.cross(v);
    }

    // Wrist center: least-squares common point of the three spherical-wrist axes.
    Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (int idx : {3, 4, 5})
    {
        const Eigen::Vector3d d = hm.col(idx);
        const Eigen::Matrix3d perp = Eigen::Matrix3d::Identity() - d * d.transpose();
        lhs += perp;
        rhs += perp * g[static_cast<std::size_t>(idx)];
    }
    const Eigen::Vector3d wrist = lhs.ldlt().solve(rhs);
    g[3] = g[4] = g[5] = wrist;

    const Eigen::Vector3d flange =
        cartan::forward_kinematics(chain, Eigen::Vector<double, 6>::Zero())
            .end_effector.translation();

    Eigen::Matrix<double, 3, 7> pm;
    pm.col(0) = g[0];
    for (int i = 1; i < 6; ++i)
        pm.col(i) = g[static_cast<std::size_t>(i)] - g[static_cast<std::size_t>(i - 1)];
    pm.col(6) = flange - g[5];

    for (int c = 0; c < 6; ++c)
        for (int r = 0; r < 3; ++r)
            h_flat[static_cast<std::size_t>(3 * c + r)] = hm(r, c);
    for (int c = 0; c < 7; ++c)
        for (int r = 0; r < 3; ++r)
            p_flat[static_cast<std::size_t>(3 * c + r)] = pm(r, c);
}

struct ikgeo_fixture
{
    kr6_chain chain;
    cartan::opw_parameters<double> params;
    kr6_solver solver;
    kr6_solver_raw solver_raw;
    std::array<double, 18> h_flat;
    std::array<double, 21> p_flat;
    std::vector<cartan::se3<double>> targets;
    std::vector<std::array<double, 9>> eerot; // row-major R_target * R_home^{-1}
    std::vector<std::array<double, 3>> eetrans;
};

ikgeo_fixture build_fixture()
{
    auto chain = cartan::fixtures::make_kr6_r900_opw_chain<double>();
    auto params = cartan::fixtures::kr6_r900_opw_parameters<double>();
    auto maybe = kr6_solver::make(chain, params);
    auto maybe_raw = kr6_solver_raw::make(chain, params);
    if (!maybe || !maybe_raw)
    {
        std::cerr << "ikgeo_comparison_benchmarks: could not construct "
                     "opw_6r_solver for the KR6 R900 fixture\n";
        std::abort();
    }

    std::array<double, 18> h_flat{};
    std::array<double, 21> p_flat{};
    make_ikgeo_hp(chain, h_flat, p_flat);

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

    return ikgeo_fixture{
        chain,      params,          std::move(*maybe), std::move(*maybe_raw),
        h_flat,     p_flat,          std::move(targets), std::move(eerot),
        std::move(eetrans)};
}

const ikgeo_fixture& fixture()
{
    static const ikgeo_fixture fx = build_fixture();
    return fx;
}

struct parity_stats
{
    long targets_solved = 0;
    long targets_skipped = 0;
    long cartan_branches = 0;
    long ikgeo_exact_solutions = 0; // non-least-squares, FK-verified
    long exact_matches = 0;
    long hard_mismatches = 0;
    long cartan_only = 0;
    long ikgeo_only = 0;
    double max_disagreement = 0.0;
    Eigen::Vector<double, 6> worst_cartan = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 6> worst_ikgeo = Eigen::Vector<double, 6>::Zero();
};

parity_stats run_parity(const ikgeo_fixture& fx)
{
    parity_stats st;
    double outq[48];
    unsigned char isls[8];

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

        const unsigned long nsol = ikgeo_spherical_two_parallel(
            fx.h_flat.data(), fx.p_flat.data(), fx.eerot[t].data(),
            fx.eetrans[t].data(), outq, isls);

        std::array<bool, 8> ig_present{};
        std::array<Eigen::Vector<double, 6>, 8> ig_q{};
        for (unsigned long k = 0; k < nsol; ++k)
        {
            if (isls[k])
                continue;
            Eigen::Vector<double, 6> q;
            for (int j = 0; j < 6; ++j)
                q(j) = outq[k * 6 + static_cast<unsigned long>(j)];

            const auto fk = cartan::forward_kinematics(fx.chain, q).end_effector;
            const double pe = (fk.translation() - target.translation()).norm();
            const double oe =
                (fk.rotation().inverse() * target.rotation()).log().norm();
            if (std::max(pe, oe) > verify_tol)
                continue;

            const int key = static_cast<int>(cartan::classify_branch(q, fx.params));
            ig_present[static_cast<std::size_t>(key)] = true;
            ig_q[static_cast<std::size_t>(key)] = q;
            ++st.ikgeo_exact_solutions;
        }

        for (std::size_t key = 0; key < 8; ++key)
        {
            const bool c = cart_present[key];
            const bool r = ig_present[key];
            if (c && r)
            {
                const double d = max_wrapped_diff(cart_q[key], ig_q[key]);
                if (d > st.max_disagreement)
                {
                    st.max_disagreement = d;
                    st.worst_cartan = cart_q[key];
                    st.worst_ikgeo = ig_q[key];
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
                ++st.ikgeo_only;
            }
        }
    }

    return st;
}

bool print_receipt(const parity_stats& st)
{
    const bool pass = st.hard_mismatches == 0 && st.cartan_only == 0
        && st.ikgeo_only == 0 && st.exact_matches > 0;

    std::cout
        << "\n=== Branch-for-branch parity: cartan opw_6r_solver vs ik-geo "
           "spherical_two_parallel (Rust) (KR6 R900) ===\n"
        << "targets solved (cartan)      : " << st.targets_solved << "\n"
        << "targets skipped (unreachable): " << st.targets_skipped << "\n"
        << "cartan branches (total)      : " << st.cartan_branches << "\n"
        << "ik-geo exact solutions       : " << st.ikgeo_exact_solutions << "\n"
        << "branches matched             : " << st.exact_matches << "\n"
        << "  (|dq| <= " << match_tol << " mod 2*pi)\n"
        << "hard mismatches              : " << st.hard_mismatches << "\n"
        << "cartan-only branches         : " << st.cartan_only << "\n"
        << "ik-geo-only branches         : " << st.ikgeo_only << "\n"
        << "max joint disagreement (rad) : "
        << std::setprecision(17) << st.max_disagreement << std::setprecision(6)
        << "\n";

    if (st.max_disagreement > match_tol)
    {
        std::cout << std::setprecision(17)
                  << "  worst cartan : " << st.worst_cartan.transpose() << "\n"
                  << "  worst ik-geo : " << st.worst_ikgeo.transpose() << "\n"
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

void bm_ikgeo_spherical_two_parallel(benchmark::State& state)
{
    const auto& fx = fixture();
    double outq[48];
    unsigned char isls[8];
    std::size_t i = 0;
    for (auto _ : state)
    {
        const std::size_t t = i % fx.targets.size();
        auto n = ikgeo_spherical_two_parallel(
            fx.h_flat.data(), fx.p_flat.data(), fx.eerot[t].data(),
            fx.eetrans[t].data(), outq, isls);
        benchmark::DoNotOptimize(n);
        benchmark::DoNotOptimize(outq);
        ++i;
    }
}
BENCHMARK(bm_ikgeo_spherical_two_parallel);

} // namespace

int main(int argc, char** argv)
{
    const bool pass = print_receipt(run_parity(fixture()));

    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();

    return pass ? 0 : 1;
}
