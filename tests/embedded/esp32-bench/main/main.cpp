/// Cartan ESP-IDF on-device IK timing sweep. Runs the native (dependency-free)
/// iterative solvers over a set of forward-kinematics-walked targets on two
/// robots, with `float` scalars, and reports per-cell convergence, cycle/us
/// timing (min/avg/max over the target set), and an independent FK re-verify of
/// each solution. Results stream two ways: a human-readable table on the console
/// UART (USB0) and machine-parseable CSV on a second UART (USB1 / external FTDI).
///
/// The telemetry UART TX pin defaults to GPIO4; set CARTAN_TELE_TX to match the
/// GPIO your FTDI RX is wired to. The sweep uses only native steppers, so it
/// reflects the allocation-free, argmin-free RT path.

#include "cartan/serial_chain.h"

#include "driver/uart.h"
#include "esp_cpu.h"
#include "esp_log.h"

#include <Eigen/Core>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <array>
#include <cmath>

namespace
{

constexpr const char* TAG = "cartan_bench";

constexpr uart_port_t k_tele_uart = UART_NUM_2;
constexpr int k_tele_tx = 17;  // ESP32 TX -> FTDI RX (external telemetry cable).
constexpr int k_tele_rx = 16;  // ESP32 RX <- FTDI TX.
constexpr float k_cpu_mhz = 160.0f;
constexpr int k_targets = 8;

void tele_init()
{
    uart_config_t cfg{};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    uart_driver_install(k_tele_uart, 256, 0, 0, nullptr, 0);
    uart_param_config(k_tele_uart, &cfg);
    uart_set_pin(k_tele_uart, k_tele_tx, k_tele_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void tele(const char* line)
{
    uart_write_bytes(k_tele_uart, line, std::strlen(line));
}

cartan::kinematic_chain<float, 3> build_planar_3r()
{
    using vec3f = cartan::vector3<float>;
    using screw = cartan::screw_axis<float>;
    auto s1 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(0.f, 0.f, 0.f));
    auto s2 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(1.f, 0.f, 0.f));
    auto s3 = screw::revolute(vec3f(0.f, 0.f, 1.f), vec3f(2.f, 0.f, 0.f));
    auto home = cartan::se3<float>(cartan::so3<float>::identity(), vec3f(3.f, 0.f, 0.f));
    cartan::joint_limits<float> lim{-3.14159265f, 3.14159265f};
    return cartan::kinematic_chain<float, 3>{home, {s1, s2, s3}, {lim, lim, lim}};
}

cartan::kinematic_chain<float, 6> build_kuka_kr6()
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
    cartan::joint_limits<float> lim{-3.14159265f, 3.14159265f};
    return cartan::kinematic_chain<float, 6>{home, {k1, k2, k3, k4, k5, k6}, {lim, lim, lim, lim, lim, lim}};
}

// Deterministic reachable joint sets (kept clear of the +-pi limits).
template <int N>
std::array<Eigen::Vector<float, N>, k_targets> make_truths()
{
    std::array<Eigen::Vector<float, N>, k_targets> out;
    for (int k = 0; k < k_targets; ++k)
    {
        for (int j = 0; j < N; ++j)
        {
            out[k][j] = 0.9f * std::sin(1.3f * static_cast<float>(k) + 0.7f * static_cast<float>(j));
        }
    }
    return out;
}

// Run one solver policy over the target set, timing each solve in CPU cycles and
// FK-re-verifying every converged solution.
template <typename Chain, typename Policy, int N>
void bench_ik(const char* robot, const char* solver, const Chain& chain,
    const std::array<Eigen::Vector<float, N>, k_targets>& truths)
{
    std::uint32_t cyc_min = UINT32_MAX;
    std::uint32_t cyc_max = 0;
    std::uint64_t cyc_sum = 0;
    int converged = 0;
    int iters_sum = 0;
    float max_reverify = 0.f;

    cartan::convergence_criteria<float> crit{};
    crit.max_iterations_per_attempt = 100;
    crit.max_total_work_units = 400;

    for (const auto& q_truth : truths)
    {
        const auto target = cartan::forward_kinematics(chain, q_truth).end_effector;
        cartan::basic_ik_runner runner{Policy{}};
        Eigen::Vector<float, N> q0 = Eigen::Vector<float, N>::Zero();
        runner.setup(chain, target, q0, crit);

        const std::uint32_t c0 = esp_cpu_get_cycle_count();
        auto result = runner.solve();
        const std::uint32_t dc = esp_cpu_get_cycle_count() - c0;

        cyc_min = std::min(cyc_min, dc);
        cyc_max = std::max(cyc_max, dc);
        cyc_sum += dc;

        if (result.has_value())
        {
            ++converged;
            iters_sum += result.value().iterations;
            const auto fk = cartan::forward_kinematics(chain, result.value().solution.position);
            const float rv = (fk.end_effector.inverse() * target).log().norm();
            max_reverify = std::max(max_reverify, rv);
        }
    }

    const std::uint32_t cyc_avg = static_cast<std::uint32_t>(cyc_sum / k_targets);
    char line[192];
    std::snprintf(line, sizeof line,
        "%-9s %-13s conv=%d/%d iter=%d cyc[min/avg/max]=%lu/%lu/%lu us[min/max]=%.1f/%.1f maxRV=%.2e",
        robot, solver, converged, k_targets, iters_sum,
        static_cast<unsigned long>(cyc_min), static_cast<unsigned long>(cyc_avg),
        static_cast<unsigned long>(cyc_max),
        static_cast<double>(cyc_min) / k_cpu_mhz, static_cast<double>(cyc_max) / k_cpu_mhz,
        static_cast<double>(max_reverify));
    ESP_LOGI(TAG, "%s", line);

    char csv[160];
    std::snprintf(csv, sizeof csv, "%s,%s,%d,%d,%d,%lu,%lu,%lu,%.4e\n",
        robot, solver, converged, k_targets, iters_sum,
        static_cast<unsigned long>(cyc_min), static_cast<unsigned long>(cyc_avg),
        static_cast<unsigned long>(cyc_max), static_cast<double>(max_reverify));
    tele(csv);
}

template <typename Chain, int N>
void bench_all_solvers(const char* robot, const Chain& chain,
    const std::array<Eigen::Vector<float, N>, k_targets>& truths)
{
    using nl = cartan::no_limits;
    bench_ik<Chain, cartan::projected_lm<Chain, nl>, N>(robot, "projected_lm", chain, truths);
    bench_ik<Chain, cartan::dls<Chain, nl>, N>(robot, "dls", chain, truths);
    bench_ik<Chain, cartan::lm<Chain, nl>, N>(robot, "lm", chain, truths);
    bench_ik<Chain, cartan::lbfgsb<Chain, nl>, N>(robot, "lbfgsb", chain, truths);
}

}

extern "C" void app_main()
{
    tele_init();
    tele("robot,solver,conv,total,iters,cyc_min,cyc_avg,cyc_max,max_reverify\n");
    ESP_LOGI(TAG, "cartan on-device IK WCET sweep @ %.0f MHz -- native float solvers, no_limits, %d targets/cell",
        static_cast<double>(k_cpu_mhz), k_targets);

    const auto c3 = build_planar_3r();
    const auto c6 = build_kuka_kr6();
    const auto t3 = make_truths<3>();
    const auto t6 = make_truths<6>();

    bench_all_solvers<cartan::kinematic_chain<float, 3>, 3>("planar3R", c3, t3);
    bench_all_solvers<cartan::kinematic_chain<float, 6>, 6>("kuka6R", c6, t6);

    tele("END\n");
    ESP_LOGI(TAG, "sweep complete");
}
