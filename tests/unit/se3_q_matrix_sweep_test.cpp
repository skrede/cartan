#ifndef HPP_GUARD_TESTS_UNIT_SE3_Q_MATRIX_SWEEP_TEST_H
#define HPP_GUARD_TESTS_UNIT_SE3_Q_MATRIX_SWEEP_TEST_H

#include <cartan/lie/se3_left_jacobian.h>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <limits>
#include <cstdio>

// ============================================================================
// SE(3) Q-matrix small-angle sweep against an independent long-double oracle.
//
// The Q-matrix coefficients c1/c2/c3 have two representations: a cancellation-
// free small-angle series (accurate near phi=0) and the exact closed form
// (accurate for larger phi but catastrophically cancelling near phi=0). This
// test pins the corrected/extended series, records where the two representations
// cross over, and locks the per-precision Taylor<->closed switch threshold with
// falsifiable sufficiency + near-optimality assertions.
//
// Oracle: the coefficient series carried to eight terms in long double. In the
// swept range phi in [3e-4, 0.3] the omitted phi^16 term is far below long-double
// epsilon, so this oracle is exact to long-double precision and is free of the
// cancellation that plagues the closed form. Per the epsilon.h long-double bug
// (its trait hands long double the *double* sqrt-epsilon), we take tolerances
// straight from raw std::numeric_limits<long double>::epsilon() here, never the
// detail::sqrt_epsilon_v<> trait.
//
// RECORDED SWEEP (max coefficient relative error vs oracle, this build; c2 is
// the limiting coefficient):
//
//   phi    | double taylor c2 | double closed c2 | float taylor c2 | float closed c2
//   0.02   | 5.0e-16          | 6.6e-09          | 4.1e-08         | 1.0e+00
//   0.04   | 2.7e-14          | 1.1e-09          | 1.4e-08         | 1.2e-01
//   0.10   | 6.6e-12          | 1.4e-11          | 8.4e-08         | 1.7e-03
//   0.20   | 4.2e-10          | 1.9e-13          | 6.9e-08         | 9.0e-04
//   0.30   | 4.8e-09          | 3.3e-13          | 8.0e-08         | 1.2e-04
//
//   double: series and closed form cross over near phi ~ 0.12 (phi_sq ~ 0.015);
//           at phi=0.10 the series (6.6e-12) still beats the closed form
//           (1.4e-11), and by phi=0.20 the closed form wins. Chosen phi_sq* = 0.01
//           (the series is active up to the crossover, both branches ~1e-11 there).
//   float:  the series is flat at ~float-eps (~1e-7) across the whole band while
//           the closed form is catastrophic (100% at phi<=0.02, 1.2e-4 at phi=0.3);
//           their crossover is far out near phi ~ 0.55. The switch therefore must
//           cover the danger band: it must exceed 0.3^2 = 0.09 so the series stays
//           active through phi=0.3. The research seed phi_sq ~ 0.06 (phi ~ 0.245)
//           is REJECTED by this sweep -- it would select the closed form at phi=0.3
//           (1.2e-4, ~120x over the 1e-6 gate). Chosen phi_sq* = 0.1.
// The full per-phi table is emitted by the "records the ... crossover" cases
// below (compile with the WARN reporter to inspect).
// ============================================================================

namespace
{

long double oracle_factorial(int n)
{
    long double f = 1.0L;
    for (int i = 2; i <= n; ++i)
        f *= static_cast<long double>(i);
    return f;
}

// Extended-Taylor oracle for the three Q-matrix coefficients (cancellation-free).
//   c1 term n: (-1)^n phi^(2n) / (2n+3)!
//   c2 term n: (-1)^n phi^(2n) / (2n+4)!
//   c3 term m: (-1)^m (m+1) phi^(2m) / (2m+5)!
cartan::detail::se3_q_coeffs<long double> oracle_coeffs(long double phi)
{
    const long double phi_sq = phi * phi;
    long double c1 = 0.0L, c2 = 0.0L, c3 = 0.0L;
    long double phi_pow = 1.0L; // phi^(2n)
    for (int n = 0; n < 8; ++n)
    {
        const long double sign = (n % 2 == 0) ? 1.0L : -1.0L;
        c1 += sign * phi_pow / oracle_factorial(2 * n + 3);
        c2 += sign * phi_pow / oracle_factorial(2 * n + 4);
        c3 += sign * static_cast<long double>(n + 1) * phi_pow / oracle_factorial(2 * n + 5);
        phi_pow *= phi_sq;
    }
    return {c1, c2, c3};
}

Eigen::Matrix<long double, 3, 3> hat_ld(const Eigen::Matrix<long double, 3, 1>& v)
{
    Eigen::Matrix<long double, 3, 3> m;
    m <<        0.0L, -v(2),  v(1),
                v(2),  0.0L, -v(0),
               -v(1),  v(0),  0.0L;
    return m;
}

// Assembled long-double Q oracle mirroring se3_Q_matrix's assembly with the
// cancellation-free oracle coefficients.
Eigen::Matrix<long double, 3, 3> oracle_Q(const Eigen::Matrix<long double, 3, 1>& omega,
                                          const Eigen::Matrix<long double, 3, 1>& rho)
{
    const long double phi = std::sqrt(omega.squaredNorm());
    const auto co = oracle_coeffs(phi);

    const Eigen::Matrix<long double, 3, 3> omega_hat = hat_ld(omega);
    const Eigen::Matrix<long double, 3, 3> rho_hat = hat_ld(rho);
    const Eigen::Matrix<long double, 3, 3> omega_sq = omega_hat * omega_hat;

    return 0.5L * rho_hat
         + co.c1 * (omega_hat * rho_hat + rho_hat * omega_hat + omega_hat * rho_hat * omega_hat)
         + co.c2 * (omega_sq * rho_hat + rho_hat * omega_sq - 3.0L * omega_hat * rho_hat * omega_hat)
         + co.c3 * (omega_sq * rho_hat * omega_hat + omega_hat * rho_hat * omega_sq);
}

template <typename Scalar>
long double rel_err(Scalar approx, long double truth)
{
    const long double denom = std::abs(truth) > 0.0L ? std::abs(truth) : 1.0L;
    return std::abs(static_cast<long double>(approx) - truth) / denom;
}

// The log-spaced phi grid spans both sides of the float and double crossovers.
constexpr double kPhiGrid[] = {3e-4, 1e-3, 4e-3, 1e-2, 2e-2, 4e-2, 0.1, 0.2, 0.3};

} // namespace

// ----------------------------------------------------------------------------
// (a) Recorded coefficient crossover table: series vs closed, each precision.
// ----------------------------------------------------------------------------
TEMPLATE_TEST_CASE("Q-matrix coefficient sweep records the series/closed crossover",
                   "[se3_q_matrix][sweep]", double, float)
{
    using Scalar = TestType;

    for (double phi_d : kPhiGrid)
    {
        const auto phi = static_cast<Scalar>(phi_d);
        const Scalar phi_sq = phi * phi;

        const auto taylor = cartan::detail::se3_q_taylor_coeffs<Scalar>(phi_sq);
        const auto closed = cartan::detail::se3_q_closed_coeffs<Scalar>(phi);
        const auto truth = oracle_coeffs(static_cast<long double>(phi_d));

        const long double t_c1 = rel_err(taylor.c1, truth.c1);
        const long double t_c2 = rel_err(taylor.c2, truth.c2);
        const long double t_c3 = rel_err(taylor.c3, truth.c3);
        const long double x_c1 = rel_err(closed.c1, truth.c1);
        const long double x_c2 = rel_err(closed.c2, truth.c2);
        const long double x_c3 = rel_err(closed.c3, truth.c3);

        char row[256];
        std::snprintf(row, sizeof(row),
                      "phi=%.4g  taylor(c1,c2,c3)=%.2Le,%.2Le,%.2Le  "
                      "closed(c1,c2,c3)=%.2Le,%.2Le,%.2Le",
                      phi_d, t_c1, t_c2, t_c3, x_c1, x_c2, x_c3);
        WARN(row);

        // Sanity floor: neither representation should be NaN/inf on the grid.
        REQUIRE(std::isfinite(static_cast<double>(taylor.c2)));
        REQUIRE(std::isfinite(static_cast<double>(closed.c2)));
    }
}

// ----------------------------------------------------------------------------
// (b) Assembled-Q float guarantee across phi in [0.02, 0.3], where the pre-fix
//     closed form (active at the old sqrt-epsilon switch) is unusable (up to
//     296% relative error). Gate recorded below.
// ----------------------------------------------------------------------------
TEST_CASE("Q-matrix assembled float error stays below the recorded gate",
          "[se3_q_matrix][sweep][float]")
{
    // Fixed representative rotation axis and translation direction.
    Eigen::Matrix<long double, 3, 1> axis_ld(0.30151134457776363L, 0.90453403373461089L, 0.30151134457776363L);
    Eigen::Matrix<long double, 3, 1> rho_ld(0.4L, -0.7L, 0.55L);

    const cartan::vector3<float> axis_f = axis_ld.cast<float>();
    const cartan::vector3<float> rho_f = rho_ld.cast<float>();

    // Recorded gate: the fixed 3-term series holds the assembled float Q well
    // below 1e-6 across this band (the pre-fix closed form here reached ~296%).
    constexpr long double kFloatQGate = 1e-6L;

    for (double phi_d : {0.02, 0.04, 0.08, 0.1, 0.15, 0.2, 0.25, 0.3})
    {
        const auto phi = static_cast<float>(phi_d);
        const cartan::vector3<float> omega_f = axis_f * phi;

        const cartan::matrix3<float> Q = cartan::se3_Q_matrix<float>(omega_f, rho_f);

        const Eigen::Matrix<long double, 3, 1> omega_ld = axis_ld * static_cast<long double>(phi_d);
        const Eigen::Matrix<long double, 3, 3> Q_truth = oracle_Q(omega_ld, rho_ld);

        long double max_abs_diff = 0.0L;
        long double max_abs_truth = 0.0L;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                max_abs_diff = std::max(max_abs_diff,
                                        std::abs(static_cast<long double>(Q(i, j)) - Q_truth(i, j)));
                max_abs_truth = std::max(max_abs_truth, std::abs(Q_truth(i, j)));
            }
        const long double q_rel = max_abs_diff / max_abs_truth;
        INFO("phi=" << phi_d << " assembled-Q float rel err=" << static_cast<double>(q_rel));
        REQUIRE(q_rel < kFloatQGate);
    }
}

// ----------------------------------------------------------------------------
// (c) c3 OBSERVABILITY GUARD. This is the assertion that makes the previously
//     invisible c3 second-order coefficient bug observable: at phi=0.05 the
//     corrected -phi^2/2520 term matches the oracle to ~2.5e-9, while the old
//     -phi^2/5040 term is off by ~3.8e-5. A <1e-7 tolerance passes on /2520 and
//     fails on /5040.
// ----------------------------------------------------------------------------
TEST_CASE("Q-matrix c3 series coefficient is observable at phi=0.05",
          "[se3_q_matrix][c3]")
{
    constexpr double phi = 0.05;
    const double phi_sq = phi * phi;

    const auto taylor = cartan::detail::se3_q_taylor_coeffs<double>(phi_sq);
    const auto truth = oracle_coeffs(static_cast<long double>(phi));

    const long double c3_rel = rel_err(taylor.c3, truth.c3);
    INFO("c3 rel err at phi=0.05: " << static_cast<double>(c3_rel));
    REQUIRE(c3_rel < 1e-7L);
}

// ----------------------------------------------------------------------------
// (d) Threshold sufficiency + near-optimality. The chosen phi_sq switch is
//     falsifiable per the empirical-sweep mandate: it must be sufficient (the
//     active branch is below the gate at the switch point) AND near-optimal (the
//     branch that is NOT chosen on each side is materially worse there, so the
//     switch sits at the crossover rather than being picked arbitrarily).
// ----------------------------------------------------------------------------
TEMPLATE_TEST_CASE("Q-matrix Taylor switch is sufficient and near-optimal",
                   "[se3_q_matrix][threshold]", double, float)
{
    using Scalar = TestType;

    const Scalar switch_phi_sq = cartan::detail::q_taylor_switch_v<Scalar>;
    const Scalar switch_phi = std::sqrt(switch_phi_sq);

    // The limiting coefficient is c2; assess sufficiency/optimality on it.
    auto c2_taylor_err = [](Scalar phi) {
        const Scalar phi_sq = phi * phi;
        return rel_err(cartan::detail::se3_q_taylor_coeffs<Scalar>(phi_sq).c2,
                       oracle_coeffs(static_cast<long double>(phi)).c2);
    };
    auto c2_closed_err = [](Scalar phi) {
        return rel_err(cartan::detail::se3_q_closed_coeffs<Scalar>(phi).c2,
                       oracle_coeffs(static_cast<long double>(phi)).c2);
    };

    if constexpr (std::is_same_v<Scalar, double>)
    {
        // Double has a real series/closed crossover just above the switch, so the
        // threshold is falsifiable by bracketing that crossover.
        const long double gate = 1e-9L;

        // (i) SUFFICIENCY: at the switch point the active (Taylor) branch is below
        //     the gate.
        REQUIRE(c2_taylor_err(switch_phi) < gate);

        // (ii) NECESSITY below the switch: a bit below the switch the closed form
        //      is materially WORSE than Taylor, so switching to closed earlier
        //      (a lower threshold) would degrade accuracy.
        const Scalar phi_below = std::sqrt(switch_phi_sq * Scalar(0.5));
        REQUIRE(c2_closed_err(phi_below) > c2_taylor_err(phi_below));

        // (iii) NECESSITY above the switch: a bit above the switch the closed form
        //       is BETTER than Taylor, so keeping Taylor past the switch (a higher
        //       threshold) would degrade accuracy. The switch sits at the crossover.
        const Scalar phi_above = std::sqrt(switch_phi_sq * Scalar(2));
        REQUIRE(c2_taylor_err(phi_above) > c2_closed_err(phi_above));
    }
    else
    {
        // Float has no crossover in the guarded band: the cancellation-free series
        // beats the catastrophic closed form everywhere up to phi ~ 0.55. The
        // switch is therefore bounded below by the coverage requirement -- it must
        // keep the series active through the top of the guaranteed band (phi=0.3).
        const long double gate = 1e-6L;
        constexpr Scalar phi_top = Scalar(0.3);

        // (i) COVERAGE: the switch keeps the series active through phi=0.3.
        REQUIRE(switch_phi_sq > phi_top * phi_top);

        // (ii) SUFFICIENCY: the active (Taylor) branch is below the gate at the top
        //      of the band.
        REQUIRE(c2_taylor_err(phi_top) < gate);

        // (iii) NECESSITY: a materially lower threshold would expose the closed form
        //       inside the band, where it violates the gate by orders of magnitude.
        REQUIRE(c2_closed_err(phi_top) > gate);
        REQUIRE(c2_closed_err(phi_top) > c2_taylor_err(phi_top));
    }
}

#endif
