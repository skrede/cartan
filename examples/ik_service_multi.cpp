/// @file ik_service_multi.cpp
/// @brief Multi-threaded IK service with cooperative multi-policy solving.
///
/// Demonstrates M worker threads processing IK requests from a shared queue.
/// Each request is solved using a variadic basic_ik_runner with two policies
/// (speed + convergence) that race cooperatively. The worker thread calls
/// solve() which internally uses step() to interleave both policies.
///
/// Unlike TRAC-IK which spawns a thread per solver strategy, cartan's
/// multi-policy basic_ik_runner runs all strategies cooperatively via step().
/// M worker threads can service many requests without thread proliferation.

#include "cartan/serial_chain.h"

#include <atomic>
#include <chrono>
#include <deque>
#include <future>
#include <iostream>
#include <mutex>
#include <numbers>
#include <random>
#include <thread>
#include <vector>
#include <condition_variable>

// --- LBR iiwa 7-DOF chain geometry (hardcoded PoE parameters) ---

cartan::expected<cartan::kinematic_chain<double, 7>, cartan::chain_failure> make_lbr_iiwa()
{
    using vec3 = cartan::vector3<double>;

    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0, 0, 0.360));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0.360));
    auto s4 = cartan::screw_axis<double>::revolute(vec3(0, -1, 0), vec3(0, 0, 0.780));
    auto s5 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0.780));
    auto s6 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0, 0, 1.180));
    auto s7 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 1.180));

    vec3 home_trans(0, 0, 1.306);
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    auto lim = cartan::joint_limits<double>::make(-std::numbers::pi, std::numbers::pi);
    if (!lim.has_value())
    {
        return cartan::unexpected(lim.error());
    }

    return cartan::kinematic_chain<double, 7>(
        home, {s1, s2, s3, s4, s5, s6, s7},
        {*lim, *lim, *lim, *lim, *lim, *lim, *lim});
}

// --- IK service types ---

struct ik_request
{
    cartan::se3<double> target;
};

struct ik_response
{
    cartan::expected<cartan::ik_result<double, 7>, cartan::ik_error<double, 7>> result;
    std::chrono::microseconds solve_time;
};

// --- Multi-threaded IK service pool ---
//
// Architecture:
//   - M worker threads pull requests from a shared queue
//   - Each request is solved by a multi-policy basic_ik_runner (speed + convergence)
//   - solve() internally calls step() which round-robins across both policies
//   - No thread-per-solver: a single worker thread drives both policies cooperatively
//
// This is the key advantage over TRAC-IK's threading model:
//   TRAC-IK: 1 thread per solver strategy -> O(strategies) threads per request
//   cartan:   M fixed worker threads -> O(M) threads total, regardless of policy count

class ik_service_pool
{
public:
    ik_service_pool(cartan::kinematic_chain<double, 7> chain, int num_workers)
        : m_chain(std::move(chain))
        , m_stop(false)
    {
        for (int i = 0; i < num_workers; ++i)
        {
            m_workers.emplace_back([this] { worker_loop(); });
        }
    }

    ~ik_service_pool()
    {
        {
            std::lock_guard lock(m_mutex);
            m_stop.store(true);
        }
        m_cv.notify_all();
        for (auto& w : m_workers)
            w.join();
    }

    /// Submit an IK request. Returns a future for the response.
    std::future<ik_response> submit(const ik_request& req)
    {
        auto promise = std::make_shared<std::promise<ik_response>>();
        auto future = promise->get_future();

        {
            std::lock_guard lock(m_mutex);
            m_queue.push_back({req, std::move(promise)});
        }
        m_cv.notify_one();

        return future;
    }

private:
    struct pending_request
    {
        ik_request request;
        std::shared_ptr<std::promise<ik_response>> promise;
    };

    void worker_loop()
    {
        cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};
        std::mt19937 rng(42);

        while (!m_stop.load())
        {
            std::unique_lock lock(m_mutex);
            m_cv.wait(lock, [this] { return !m_queue.empty() || m_stop.load(); });

            if (m_stop.load())
                break;

            pending_request item = std::move(m_queue.front());
            m_queue.pop_front();
            lock.unlock();

            auto start = std::chrono::steady_clock::now();

            // Multi-policy solver: two policies race cooperatively.
            // step() advances each active policy once per call (round-robin).
            // No additional threads -- cooperative scheduling in this worker thread.
            auto solver = cartan::make_dual_ik_runner<cartan::kinematic_chain<double, 7>>().build();

            // Generate random initial configuration
            Eigen::Vector<double, 7> q0;
            for (int j = 0; j < 7; ++j)
            {
                auto lo = m_chain.limits()[static_cast<std::size_t>(j)].position_min();
                auto hi = m_chain.limits()[static_cast<std::size_t>(j)].position_max();
                std::uniform_real_distribution<double> dist(lo, hi);
                q0(j) = dist(rng);
            }

            solver.setup(m_chain, item.request.target, q0, criteria);
            auto result = solver.solve();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - start);

            item.promise->set_value({std::move(result), elapsed});
        }
    }

    cartan::kinematic_chain<double, 7> m_chain;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<pending_request> m_queue;
    // std::jthread would carry the stop flag and the join, but libc++ still
    // ships <stop_token> behind an experimental opt-in, so an example meant to
    // build on every supported toolchain owns both explicitly.
    std::atomic<bool> m_stop;
    std::vector<std::thread> m_workers;
};

int main()
{
    auto chain = make_lbr_iiwa();
    if (!chain.has_value())
    {
        std::cerr << "chain construction failed: "
                  << cartan::message(chain.error()) << "\n";
        return 1;
    }

    constexpr int num_workers = 4;
    constexpr int num_requests = 10;

    ik_service_pool pool(*chain, num_workers);

    // Generate targets via FK at known configurations
    std::vector<Eigen::Vector<double, 7>> configs = {
        {0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3},
        {-0.1, 0.5, -0.3, 0.7, -0.2, 0.4, -0.1},
        {0.4, -0.2, 0.5, -0.3, 0.1, -0.4, 0.2},
        {-0.3, 0.1, -0.2, 0.4, -0.5, 0.3, -0.2},
        {0.1, -0.4, 0.3, -0.6, 0.2, -0.1, 0.4},
        {-0.2, 0.3, -0.1, 0.5, -0.3, 0.2, -0.3},
        {0.3, -0.1, 0.4, -0.2, 0.5, -0.3, 0.1},
        {-0.4, 0.2, -0.5, 0.3, -0.1, 0.4, -0.2},
        {0.5, -0.5, 0.2, -0.4, 0.3, -0.2, 0.5},
        {-0.1, 0.4, -0.3, 0.2, -0.4, 0.1, -0.1},
    };

    // Submit all requests concurrently
    std::vector<std::future<ik_response>> futures;
    futures.reserve(num_requests);

    for (int i = 0; i < num_requests; ++i)
    {
        auto fk = cartan::forward_kinematics(
            *chain, configs[static_cast<std::size_t>(i)]);
        if (!fk.has_value())
        {
            std::cerr << "forward kinematics rejected configuration " << i << ": "
                      << cartan::message(fk.error()) << "\n";
            return 1;
        }

        futures.push_back(pool.submit({fk->end_effector}));
    }

    // Collect and print results
    std::cout << "IK Service Pool: " << num_workers << " workers, "
              << num_requests << " requests\n\n";

    for (int i = 0; i < num_requests; ++i)
    {
        auto response = futures[static_cast<std::size_t>(i)].get();

        if (response.result.has_value())
        {
            auto& r = *response.result;
            std::cout << "Request " << i << ": converged in "
                      << r.iterations << " steps (policy "
                      << r.solver_index << "), "
                      << response.solve_time.count() << " us\n";
        }
        else
        {
            std::cout << "Request " << i << ": failed, "
                      << response.solve_time.count() << " us\n";
        }
    }

    return 0;
}
