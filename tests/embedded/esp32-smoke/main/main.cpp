/// Cartan ESP-IDF on-device validation. Runs a representative slice of the
/// public surface on real silicon with `float` scalars and streams the results
/// plus rough cycle/microsecond timing over the console UART, so the numbers can
/// be diffed against the host reference. Exercises: Lie groups, a small
/// kinematic chain, forward kinematics, a closed-form Paden-Kahan subproblem,
/// and a full iterative IK solve with the allocation-free projected_lm stepper.

#include "cartan/serial_chain.h"
#include "cartan/analytical/paden_kahan.h"

#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_log.h"

#include <Eigen/Core>

#include <cstdint>

namespace
{

constexpr const char* TAG = "cartan_smoke";

// Exceptions are off here, so a refusal cannot be thrown and must not be read
// through expected's accessor either: it is propagated to app_main, which has a
// console to report it on.
template <int N>
using chain_or_failure =
    cartan::expected<cartan::kinematic_chain<float, N>, cartan::chain_failure>;

chain_or_failure<3> build_planar_3r()
{
    using vec3f = cartan::vector3<float>;
    using screw = cartan::screw_axis<float>;

    auto s1 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(0.f, 0.f, 0.f));
    auto s2 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(1.f, 0.f, 0.f));
    auto s3 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(2.f, 0.f, 0.f));

    using rot = cartan::so3<float>;
    using pose = cartan::se3<float>;
    pose home(rot::identity(), vec3f(3.f, 0.f, 0.f));

    auto lim = cartan::joint_limits<float>::make(-3.14f, 3.14f);
    if (!lim.has_value())
    {
        return cartan::unexpected(lim.error());
    }
    return cartan::kinematic_chain<float, 3>{home, {s1, s2, s3}, {*lim, *lim, *lim}};
}

// KUKA KR 6 R900 SIXX space-frame screw axes at the home pose (meters).
chain_or_failure<6> build_kuka_kr6()
{
    using vec3f = cartan::vector3<float>;
    using screw = cartan::screw_axis<float>;

    auto k1 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(0.f,    0.f, 0.f));
    auto k2 = screw::revolute(vec3f(0.f, 1.f, 0.f), vec3f(0.f,    0.f, 0.400f));
    auto k3 = screw::revolute(vec3f(0.f, 1.f, 0.f), vec3f(0.455f, 0.f, 0.400f));
    auto k4 = screw::revolute(vec3f(1.f, 0.f, 0.f), vec3f(0.875f, 0.f, 0.400f));
    auto k5 = screw::revolute(vec3f(0.f, 1.f, 0.f), vec3f(0.875f, 0.f, 0.400f));
    auto k6 = screw::revolute(vec3f(1.f, 0.f, 0.f), vec3f(0.935f, 0.f, 0.400f));

    auto home = cartan::se3<float>(cartan::so3<float>::identity(), vec3f(0.935f, 0.f, 0.400f));
    auto lim = cartan::joint_limits<float>::make(-3.14159265f, 3.14159265f);
    if (!lim.has_value())
    {
        return cartan::unexpected(lim.error());
    }
    return cartan::kinematic_chain<float, 6>{
        home, {k1, k2, k3, k4, k5, k6}, {*lim, *lim, *lim, *lim, *lim, *lim}};
}

/// Planar 3R: forward kinematics and one closed-form subproblem.
void run_planar_3r_slice()
{
    auto chain = build_planar_3r();
    if (!chain.has_value())
    {
        ESP_LOGE(TAG, "planar-3R chain refused: %s", cartan::message(chain.error()));
        return;
    }

    Eigen::Matrix<float, 3, 1> q;
    q << 0.1f, 0.2f, 0.3f;

    auto fk = cartan::forward_kinematics(*chain, q);
    if (!fk.has_value())
    {
        ESP_LOGE(TAG, "planar-3R FK refused: %s", cartan::message(fk.error()));
        return;
    }
    ESP_LOGI(TAG, "planar-3R FK end-effector x = %.4f (host = 2.7757)",
        static_cast<double>(fk->end_effector.translation().x()));

    cartan::vector3<float> omega(0.f, 0.f, 1.f);
    cartan::vector3<float> q_axis(0.f, 0.f, 0.f);
    cartan::vector3<float> p(1.f, 0.f, 0.f);
    cartan::vector3<float> p_prime(0.f, 1.f, 0.f);
    auto pk1 = cartan::paden_kahan_1<float>(omega, q_axis, p, p_prime);
    if (pk1.has_value())
    {
        ESP_LOGI(TAG, "paden_kahan_1 theta = %.4f (host = 1.5708)",
            static_cast<double>(*pk1));
    }
}

/// Forward-kinematics throughput. The probe configuration is fixed for the
/// whole sweep, so its shape and finiteness are established once by the caller
/// and the loops here call the unchecked sibling -- the shape a real-time
/// consumer wants, and the reason the number below measures forward kinematics
/// rather than the guard in front of it. The sink is reported here rather than
/// at the end of the run so that nothing downstream can suppress it.
void report_fk_throughput(const cartan::kinematic_chain<float, 6>& chain,
    const Eigen::Vector<float, 6>& q_probe)
{
    volatile float sink = 0.f;
    for (int i = 0; i < 16; ++i)
    {
        sink += cartan::forward_kinematics_unchecked(chain, q_probe)
                    .end_effector.translation().x();
    }
    constexpr int fk_iters = 1000;
    const uint32_t fk_c0 = esp_cpu_get_cycle_count();
    const int64_t fk_t0 = esp_timer_get_time();
    for (int i = 0; i < fk_iters; ++i)
    {
        sink += cartan::forward_kinematics_unchecked(chain, q_probe)
                    .end_effector.translation().x();
    }
    const int64_t fk_t1 = esp_timer_get_time();
    const uint32_t fk_c1 = esp_cpu_get_cycle_count();

    ESP_LOGI(TAG, "6R FK: %.3f us/call, %lu cycles/call (avg of %d)",
        static_cast<double>(fk_t1 - fk_t0) / fk_iters,
        static_cast<unsigned long>((fk_c1 - fk_c0) / fk_iters), fk_iters);
    ESP_LOGI(TAG, "fk-timing sink = %.4f", static_cast<double>(sink));
}

void report_ik_solution(const cartan::kinematic_chain<float, 6>& chain,
    const cartan::se3<float>& target, const cartan::ik_result<float, 6>& r,
    std::int64_t solve_us, std::uint32_t solve_cycles)
{
    // FK re-verify: independent body-twist error, never trust self-report.
    auto fk_check = cartan::forward_kinematics(chain, r.solution.position);
    if (!fk_check.has_value())
    {
        ESP_LOGE(TAG, "6R IK re-verify refused: %s", cartan::message(fk_check.error()));
        return;
    }
    const float reverify = (fk_check->end_effector.inverse() * target).log().norm();

    ESP_LOGI(TAG, "6R IK: CONVERGED  iters=%d  self_err=%.7f  reverify_err=%.7f",
        r.iterations, static_cast<double>(r.final_error_norm),
        static_cast<double>(reverify));
    ESP_LOGI(TAG, "6R IK: solve %.1f us, %lu cycles",
        static_cast<double>(solve_us), static_cast<unsigned long>(solve_cycles));
    ESP_LOGI(TAG, "6R IK: q[0..2] = %.4f %.4f %.4f",
        static_cast<double>(r.solution.position[0]),
        static_cast<double>(r.solution.position[1]),
        static_cast<double>(r.solution.position[2]));
    ESP_LOGI(TAG, "6R IK: q[3..5] = %.4f %.4f %.4f",
        static_cast<double>(r.solution.position[3]),
        static_cast<double>(r.solution.position[4]),
        static_cast<double>(r.solution.position[5]));
}

/// 6R KUKA: forward-kinematics timing, one iterative solve, and a re-verify.
void run_kuka_6r_slice()
{
    using Chain6 = cartan::kinematic_chain<float, 6>;
    auto chain6_held = build_kuka_kr6();
    if (!chain6_held.has_value())
    {
        ESP_LOGE(TAG, "6R chain refused: %s", cartan::message(chain6_held.error()));
        return;
    }
    const Chain6& chain6 = *chain6_held;

    Eigen::Vector<float, 6> q_truth;
    q_truth << 0.2f, -0.4f, 0.3f, -0.5f, 0.6f, -0.2f;
    Eigen::Vector<float, 6> q_probe;
    q_probe << 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f;

    auto target_fk = cartan::forward_kinematics(chain6, q_truth);
    auto probe_fk = cartan::forward_kinematics(chain6, q_probe);
    if (!target_fk.has_value() || !probe_fk.has_value())
    {
        ESP_LOGE(TAG, "6R setup FK refused: %s", cartan::message(
            target_fk.has_value() ? probe_fk.error() : target_fk.error()));
        return;
    }
    const auto target = target_fk->end_effector;

    report_fk_throughput(chain6, q_probe);

    // IK solve with the allocation-free projected_lm stepper (self-restarting),
    // from a zero seed to the FK-walked target.
    cartan::basic_ik_runner solver{cartan::projected_lm<Chain6, cartan::no_limits>{}};
    Eigen::Vector<float, 6> q0 = Eigen::Vector<float, 6>::Zero();
    cartan::convergence_criteria<float> criteria{};
    criteria.max_iterations_per_attempt = 100;
    criteria.max_total_work_units = 400;

    solver.setup(chain6, target, q0, criteria);
    const uint32_t ik_c0 = esp_cpu_get_cycle_count();
    const int64_t ik_t0 = esp_timer_get_time();
    auto result = solver.solve();
    const int64_t ik_t1 = esp_timer_get_time();
    const uint32_t ik_c1 = esp_cpu_get_cycle_count();

    if (!result.has_value())
    {
        ESP_LOGI(TAG, "6R IK: FAILED  reason=%d  last_err=%.7f",
            static_cast<int>(result.error().reason),
            static_cast<double>(result.error().last_error_norm));
        return;
    }
    report_ik_solution(chain6, target, *result, ik_t1 - ik_t0, ik_c1 - ik_c0);
}

}

extern "C" void app_main()
{
    run_planar_3r_slice();
    run_kuka_6r_slice();
}
