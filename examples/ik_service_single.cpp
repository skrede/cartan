/// @file ik_service_single.cpp
/// @brief Single-threaded IK service with CV-based worker dispatch.
///
/// Demonstrates a worker thread that waits on a condition_variable for
/// IK requests, solves them using cartan's restart_wrapper, and returns results.
/// The main thread submits requests and collects responses.

#include "cartan/serial_chain.h"

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <numbers>
#include <optional>
#include <queue>
#include <thread>

// --- UR3e 6-DOF chain geometry (hardcoded PoE parameters) ---

cartan::expected<cartan::kinematic_chain<double, 6>, cartan::chain_failure> make_ur3e()
{
    using vec3 = cartan::vector3<double>;

    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0, 0, 0.15185));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.24355, 0, 0.15185));
    auto s4 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.45675, 0, 0.15185));
    auto s5 = cartan::screw_axis<double>::revolute(vec3(0, 0, -1), vec3(-0.45675, 0.13105, 0));
    auto s6 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.45675, 0, -0.08535));

    vec3 home_trans(-0.45675, 0.22315, 0.0665);
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    auto lim = cartan::joint_limits<double>::make(-std::numbers::pi, std::numbers::pi);
    if (!lim.has_value())
    {
        return cartan::unexpected(lim.error());
    }

    return cartan::kinematic_chain<double, 6>(
        home, {s1, s2, s3, s4, s5, s6},
        {*lim, *lim, *lim, *lim, *lim, *lim});
}

// --- IK service types ---

struct ik_request
{
    cartan::se3<double> target;
    Eigen::Vector<double, 6> q0;
};

struct ik_response
{
    cartan::expected<cartan::ik_result<double, 6>, cartan::ik_error<double, 6>> result;
};

// --- Single-threaded IK service ---

class ik_service
{
public:
    explicit ik_service(cartan::kinematic_chain<double, 6> chain)
        : m_chain(std::move(chain))
        , m_stop(false)
        , m_worker([this] { worker_loop(); })
    {
    }

    ~ik_service()
    {
        {
            std::lock_guard lock(m_mutex);
            m_stop.store(true);
        }
        m_cv.notify_one();
        m_worker.join();
    }

    /// Submit an IK request and block until the result is ready.
    ik_response solve(const ik_request& req)
    {
        // Enqueue request
        {
            std::lock_guard lock(m_mutex);
            m_requests.push(req);
        }
        m_cv.notify_one();

        // Wait for response
        std::unique_lock lock(m_mutex);
        m_response_cv.wait(lock, [this] { return m_response.has_value(); });
        ik_response resp = std::move(m_response.value());
        m_response.reset();
        return resp;
    }

private:
    void worker_loop()
    {
        cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

        while (!m_stop.load())
        {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [this] { return !m_requests.empty() || m_stop.load(); });

            if (m_stop.load())
                break;

            ik_request req = m_requests.front();
            m_requests.pop();
            lock.unlock();

            // Solve IK using LM solve policy
            cartan::basic_ik_runner<cartan::restart_wrapper<cartan::kinematic_chain<double, 6>>> solver;
            solver.setup(m_chain, req.target, req.q0, criteria);
            auto result = solver.solve();

            // Post response
            {
                std::lock_guard resp_lock(m_mutex);
                m_response = ik_response{std::move(result)};
            }
            m_response_cv.notify_one();
        }
    }

    cartan::kinematic_chain<double, 6> m_chain;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::condition_variable m_response_cv;
    std::queue<ik_request> m_requests;
    std::optional<ik_response> m_response;
    // std::jthread would carry the stop flag and the join, but libc++ still
    // ships <stop_token> behind an experimental opt-in, so an example meant to
    // build on every supported toolchain owns both explicitly.
    std::atomic<bool> m_stop;
    std::thread m_worker;
};

int main()
{
    auto chain = make_ur3e();
    if (!chain.has_value())
    {
        std::cerr << "chain construction failed: "
                  << cartan::message(chain.error()) << "\n";
        return 1;
    }

    ik_service service(*chain);

    // Generate targets via FK at known configurations
    std::array<Eigen::Vector<double, 6>, 4> configs = {{
        {0.2, -0.3, 0.5, -0.4, 0.1, 0.3},
        {-0.1, 0.4, -0.2, 0.6, -0.3, 0.1},
        {0.5, -0.1, 0.3, -0.2, 0.4, -0.5},
        {-0.3, 0.2, -0.4, 0.3, -0.1, 0.2},
    }};

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();

    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        auto fk = cartan::forward_kinematics(*chain, configs[i]);
        if (!fk.has_value())
        {
            std::cerr << "forward kinematics rejected configuration " << i << ": "
                      << cartan::message(fk.error()) << "\n";
            return 1;
        }

        auto response = service.solve({fk->end_effector, q0});

        if (response.result.has_value())
        {
            auto& r = *response.result;
            std::cout << "Request " << i << ": converged in "
                      << r.iterations << " iterations, error = "
                      << r.final_error_norm << "\n";
        }
        else
        {
            std::cout << "Request " << i << ": failed\n";
        }
    }

    return 0;
}
