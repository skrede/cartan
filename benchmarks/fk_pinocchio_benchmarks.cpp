#include "benchmark_utils.h"

#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/forward_kinematics_matrix.h>
#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/fk/jacobian_matrix.h>
#include <cartan/serial/fk/detail/axis_specializations.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <benchmark/benchmark.h>

#include <array>
#include <random>
#include <string>
#include <cstddef>
#include <stdexcept>

namespace
{

// Cartan-side cells draw their config from a table of varied inputs indexed by
// the iteration counter and DoNotOptimize the chosen input before the op, so a
// loop-invariant result cannot be hoisted out of the timed loop. The Pinocchio
// cell keeps its own loop-invariant q (equal end-effector work is the point).
// Power-of-two size wraps the index with a mask.
constexpr std::size_t kInputs = 1024;

template <typename Scalar>
auto make_3r_planar_static()
{
    auto kc = cartan::fixtures::make_3r_planar_chain<Scalar>();
    return cartan::static_chain<Scalar, cartan::revolute_z, cartan::revolute_z, cartan::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_ur3e_static()
{
    auto kc = cartan::fixtures::make_ur3e_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_y, cartan::revolute_z, cartan::revolute_y>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_lbr_med14_static()
{
    auto kc = cartan::fixtures::make_lbr_med14_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z, cartan::revolute_y,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_kr6_sixx_static()
{
    auto kc = cartan::fixtures::make_kr6_sixx_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_panda_static()
{
    auto kc = cartan::fixtures::make_panda_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z, cartan::revolute_y,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_abb_irb120_static()
{
    auto kc = cartan::fixtures::make_abb_irb120_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_jaco2_static()
{
    auto kc = cartan::fixtures::make_jaco2_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_fetch_static()
{
    auto kc = cartan::fixtures::make_fetch_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_x, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_baxter_static()
{
    auto kc = cartan::fixtures::make_baxter_chain<Scalar>();
    return cartan::static_chain<Scalar,
        cartan::revolute_z, cartan::revolute_y, cartan::revolute_x, cartan::revolute_y,
        cartan::revolute_x, cartan::revolute_y, cartan::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

// ============================================================================
// Cartan PoE -> Pinocchio Model converter
// ============================================================================
//
// Cartan stores each joint's screw axis (omega, v) in the *space* (base) frame
// at home configuration q=0. For a revolute joint, the closest point on the
// axis to the origin is q_perp = omega x v.
//
// Pinocchio uses parent-relative joint placements with a per-joint local axis.
// Since every cartan home pose in the test set has identity rotation and we
// keep all joint frames identity-rotated, parent-to-child joint placements are
// pure translations between consecutive joint axis points, and each joint's
// local axis equals its world axis.
//
//   T_{i-1,i} = (I, q_i_perp - q_{i-1}_perp)
//   joint i axis = omega_i (in world == in joint frame, since rotations are I)
//   T_{n, ee}   = (home.rotation, home.translation - q_n_perp)
//
// The bench fails fast at startup if FK from the converted Pinocchio model
// disagrees with cartan FK at a sample configuration.

struct pinocchio_chain
{
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex ee_frame_id{};

    explicit pinocchio_chain(pinocchio::Model m)
        : model(std::move(m)), data(model)
    {
    }
};

template <typename Chain>
pinocchio_chain build_pinocchio_chain(const Chain& chain, const std::string& name)
{
    using Scalar = typename Chain::scalar_type;
    using vec3 = Eigen::Matrix<Scalar, 3, 1>;

    pinocchio::Model model;
    model.name = name;

    pinocchio::JointIndex parent_id = 0;
    vec3 prev_axis_point = vec3::Zero();

    const int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        const auto& s = chain.axis(i);
        if (!s.is_revolute())
        {
            throw std::runtime_error("pinocchio chain builder: prismatic joints not supported");
        }

        vec3 omega = s.omega();
        vec3 v = s.v();
        vec3 q_perp = omega.cross(v); // closest point on screw axis to origin

        pinocchio::SE3 placement;
        placement.setIdentity();
        placement.translation() = q_perp - prev_axis_point;

        parent_id = model.addJoint(
            parent_id,
            pinocchio::JointModelRevoluteUnaligned(omega.template cast<double>()),
            placement,
            "j" + std::to_string(i));

        prev_axis_point = q_perp;
    }

    // EE frame on the last joint: home.rotation, t = home.translation - q_n
    const auto& home = chain.home();
    pinocchio::SE3 ee_placement;
    ee_placement.rotation() = home.rotation().matrix().template cast<double>();
    ee_placement.translation() = home.translation().template cast<double>() - prev_axis_point.template cast<double>();

    pinocchio_chain pc(std::move(model));
    pc.ee_frame_id = pc.model.addFrame(pinocchio::Frame(
        "ee", parent_id, ee_placement, pinocchio::OP_FRAME));
    pc.data = pinocchio::Data(pc.model);
    return pc;
}

template <typename Chain>
Eigen::VectorXd cartan_q_to_pinocchio(const Chain& chain, const auto& q_cartan)
{
    Eigen::VectorXd q(chain.num_joints());
    for (int i = 0; i < chain.num_joints(); ++i)
        q(i) = static_cast<double>(q_cartan(i));
    return q;
}

template <typename ChainType>
auto random_config_static(const ChainType& chain, std::mt19937& rng)
{
    using Scalar = typename ChainType::scalar_type;
    constexpr int N = ChainType::joints;
    using position_type = typename cartan::joint_state<Scalar, N>::position_type;

    position_type q;
    const auto& limits = chain.limits();
    for (int i = 0; i < N; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        std::uniform_real_distribution<Scalar> dist(
            limits[idx].position_min, limits[idx].position_max);
        q(i) = dist(rng);
    }
    return q;
}

// Sanity check: pinocchio FK must agree with cartan FK at one config.
template <typename Chain>
void verify_equivalence(const Chain& chain, pinocchio_chain& pc, const std::string& robot)
{
    std::mt19937 rng(123);
    auto q_cartan = random_config_static(chain, rng);
    auto pose_cartan = cartan::forward_kinematics(chain, q_cartan);

    Eigen::VectorXd q_pin = cartan_q_to_pinocchio(chain, q_cartan);
    pinocchio::framesForwardKinematics(pc.model, pc.data, q_pin);
    const auto& M_pin = pc.data.oMf[pc.ee_frame_id];

    Eigen::Matrix3d R_cartan = pose_cartan.end_effector.rotation().matrix().template cast<double>();
    Eigen::Vector3d t_cartan = pose_cartan.end_effector.translation().template cast<double>();
    double r_err = (M_pin.rotation() - R_cartan).norm();
    double t_err = (M_pin.translation() - t_cartan).norm();
    if (r_err > 1e-9 || t_err > 1e-9)
    {
        throw std::runtime_error(
            "pinocchio/cartan FK disagree on " + robot
            + " — rotation err " + std::to_string(r_err)
            + ", translation err " + std::to_string(t_err));
    }
}

// ============================================================================
// Benchmark macros
// ============================================================================

// Experimental matrix-accumulator FK: takes cartan kinematic_chain, accumulates
// rotation as Eigen::Matrix3 inside the loop. exp_joint still returns quaternion
// form, so we pay quat->mat conversion per step. This is the WORST case for
// matrix-form accumulation; a hypothetical exp_joint that returns matrix-form
// rotation directly (using axis-aligned sparsity) would skip that conversion.
// The bench answers: even paying the conversion, does matrix compose beat
// quaternion compose in the FK accumulator?
template <typename Chain>
auto fk_matrix_accum(const Chain& chain, const auto& q)
{
    using Scalar = typename Chain::scalar_type;
    Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();
    Eigen::Vector3<Scalar> t = Eigen::Vector3<Scalar>::Zero();
    int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        auto step = cartan::detail::exp_joint_runtime(
            chain.kind(i), q(i), chain.axis(i));
        Eigen::Matrix3<Scalar> R_step = step.rotation().matrix();
        Eigen::Vector3<Scalar> t_step = step.translation();
        t.noalias() = R * t_step + t;
        Eigen::Matrix3<Scalar> R_new;
        R_new.noalias() = R * R_step;
        R = R_new;
    }
    auto R_home = chain.home().rotation().matrix();
    auto t_home = chain.home().translation();
    Eigen::Vector3<Scalar> t_out;
    t_out.noalias() = R * t_home + t;
    Eigen::Matrix3<Scalar> R_out;
    R_out.noalias() = R * R_home;
    return std::pair{R_out, t_out};
}

// Build axis-aligned rotation matrix and translation directly without going
// through quaternion. Skips the quat->matrix conversion. Same trig and
// translation algebra as exp_joint but rotation lands as 3x3 matrix.
template <typename Scalar>
inline void exp_joint_matrix_z(Scalar q, const cartan::screw_axis<Scalar>& axis,
                                Eigen::Matrix3<Scalar>& R, Eigen::Vector3<Scalar>& t)
{
    Scalar s_ax = axis.omega()(2);
    Scalar theta = s_ax * q;
    Scalar half_theta = theta / Scalar(2);
    Scalar sin_h, cos_h;
    cartan::detail::fk_sincos(half_theta, sin_h, cos_h);
    Scalar sin_t = Scalar(2) * sin_h * cos_h;
    Scalar cos_t = Scalar(1) - Scalar(2) * sin_h * sin_h;
    R << cos_t, -sin_t, Scalar(0),
         sin_t,  cos_t, Scalar(0),
         Scalar(0), Scalar(0), Scalar(1);
    Scalar theta_sq = theta * theta;
    Scalar sinc, omcc;
    if (theta_sq < cartan::detail::epsilon_v<Scalar>) {
        sinc = Scalar(1) - theta_sq / Scalar(6);
        omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
    } else {
        Scalar sinc_h = sin_h / half_theta;
        sinc = sinc_h * cos_h;
        omcc = sinc_h * sin_h;
    }
    auto rho = (axis.v() * q).eval();
    t(0) = sinc * rho(0) - omcc * rho(1);
    t(1) = omcc * rho(0) + sinc * rho(1);
    t(2) = rho(2);
}

template <typename Scalar>
inline void exp_joint_matrix_y(Scalar q, const cartan::screw_axis<Scalar>& axis,
                                Eigen::Matrix3<Scalar>& R, Eigen::Vector3<Scalar>& t)
{
    Scalar s_ax = axis.omega()(1);
    Scalar theta = s_ax * q;
    Scalar half_theta = theta / Scalar(2);
    Scalar sin_h, cos_h;
    cartan::detail::fk_sincos(half_theta, sin_h, cos_h);
    Scalar sin_t = Scalar(2) * sin_h * cos_h;
    Scalar cos_t = Scalar(1) - Scalar(2) * sin_h * sin_h;
    R << cos_t,  Scalar(0), sin_t,
         Scalar(0), Scalar(1), Scalar(0),
        -sin_t,  Scalar(0), cos_t;
    Scalar theta_sq = theta * theta;
    Scalar sinc, omcc;
    if (theta_sq < cartan::detail::epsilon_v<Scalar>) {
        sinc = Scalar(1) - theta_sq / Scalar(6);
        omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
    } else {
        Scalar sinc_h = sin_h / half_theta;
        sinc = sinc_h * cos_h;
        omcc = sinc_h * sin_h;
    }
    auto rho = (axis.v() * q).eval();
    t(0) = sinc * rho(0) + omcc * rho(2);
    t(1) = rho(1);
    t(2) = -omcc * rho(0) + sinc * rho(2);
}

template <typename Scalar>
inline void exp_joint_matrix_x(Scalar q, const cartan::screw_axis<Scalar>& axis,
                                Eigen::Matrix3<Scalar>& R, Eigen::Vector3<Scalar>& t)
{
    Scalar s_ax = axis.omega()(0);
    Scalar theta = s_ax * q;
    Scalar half_theta = theta / Scalar(2);
    Scalar sin_h, cos_h;
    cartan::detail::fk_sincos(half_theta, sin_h, cos_h);
    Scalar sin_t = Scalar(2) * sin_h * cos_h;
    Scalar cos_t = Scalar(1) - Scalar(2) * sin_h * sin_h;
    R << Scalar(1), Scalar(0), Scalar(0),
         Scalar(0), cos_t, -sin_t,
         Scalar(0), sin_t,  cos_t;
    Scalar theta_sq = theta * theta;
    Scalar sinc, omcc;
    if (theta_sq < cartan::detail::epsilon_v<Scalar>) {
        sinc = Scalar(1) - theta_sq / Scalar(6);
        omcc = theta / Scalar(2) - theta * theta_sq / Scalar(24);
    } else {
        Scalar sinc_h = sin_h / half_theta;
        sinc = sinc_h * cos_h;
        omcc = sinc_h * sin_h;
    }
    auto rho = (axis.v() * q).eval();
    t(0) = rho(0);
    t(1) = sinc * rho(1) - omcc * rho(2);
    t(2) = omcc * rho(1) + sinc * rho(2);
}

// Native matrix-form FK: builds rotation matrices directly from the cached
// joint_kind on each kinematic_chain step. No quaternion in the hot loop.
// NOTE: This variant does NOT store per-joint intermediates (Jacobian-ready).
// For an apples-to-apples comparison vs cartan::forward_kinematics use
// fk_matrix_native_full (below) which mirrors the intermediate storage.
template <typename Scalar, int N>
auto fk_matrix_native(const cartan::kinematic_chain<Scalar, N>& chain,
                       const typename cartan::joint_state<Scalar, N>::position_type& q)
{
    Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();
    Eigen::Vector3<Scalar> t = Eigen::Vector3<Scalar>::Zero();
    Eigen::Matrix3<Scalar> R_step;
    Eigen::Vector3<Scalar> t_step;
    int n = chain.num_joints();
    for (int i = 0; i < n; ++i) {
        auto kind = chain.kind(i);
        const auto& axis = chain.axis(i);
        switch (kind) {
            case cartan::joint_kind::revolute_x: exp_joint_matrix_x(q(i), axis, R_step, t_step); break;
            case cartan::joint_kind::revolute_y: exp_joint_matrix_y(q(i), axis, R_step, t_step); break;
            case cartan::joint_kind::revolute_z: exp_joint_matrix_z(q(i), axis, R_step, t_step); break;
            default: {
                auto se = cartan::detail::exp_joint_runtime(kind, q(i), axis);
                R_step = se.rotation().matrix();
                t_step = se.translation();
            }
        }
        Eigen::Vector3<Scalar> t_new;
        t_new.noalias() = R * t_step + t;
        Eigen::Matrix3<Scalar> R_new;
        R_new.noalias() = R * R_step;
        t = t_new;
        R = R_new;
    }
    auto R_home = chain.home().rotation().matrix();
    auto t_home = chain.home().translation();
    Eigen::Vector3<Scalar> t_out;
    t_out.noalias() = R * t_home + t;
    Eigen::Matrix3<Scalar> R_out;
    R_out.noalias() = R * R_home;
    return std::pair{R_out, t_out};
}

// Same as fk_matrix_native but stores per-joint cumulative SE3 intermediates
// in matrix-form (as a std::array of pair<Matrix3, Vector3>) for Jacobian use.
template <typename Scalar, int N>
auto fk_matrix_native_full(const cartan::kinematic_chain<Scalar, N>& chain,
                            const typename cartan::joint_state<Scalar, N>::position_type& q)
{
    using PoseMat = std::pair<Eigen::Matrix3<Scalar>, Eigen::Vector3<Scalar>>;
    std::array<PoseMat, static_cast<std::size_t>(N)> intermediates;

    Eigen::Matrix3<Scalar> R = Eigen::Matrix3<Scalar>::Identity();
    Eigen::Vector3<Scalar> t = Eigen::Vector3<Scalar>::Zero();
    Eigen::Matrix3<Scalar> R_step;
    Eigen::Vector3<Scalar> t_step;
    int n = chain.num_joints();
    for (int i = 0; i < n; ++i) {
        auto kind = chain.kind(i);
        const auto& axis = chain.axis(i);
        switch (kind) {
            case cartan::joint_kind::revolute_x: exp_joint_matrix_x(q(i), axis, R_step, t_step); break;
            case cartan::joint_kind::revolute_y: exp_joint_matrix_y(q(i), axis, R_step, t_step); break;
            case cartan::joint_kind::revolute_z: exp_joint_matrix_z(q(i), axis, R_step, t_step); break;
            default: {
                auto se = cartan::detail::exp_joint_runtime(kind, q(i), axis);
                R_step = se.rotation().matrix();
                t_step = se.translation();
            }
        }
        Eigen::Vector3<Scalar> t_new;
        t_new.noalias() = R * t_step + t;
        Eigen::Matrix3<Scalar> R_new;
        R_new.noalias() = R * R_step;
        t = t_new;
        R = R_new;
        intermediates[static_cast<std::size_t>(i)] = {R, t};
    }
    auto R_home = chain.home().rotation().matrix();
    auto t_home = chain.home().translation();
    Eigen::Vector3<Scalar> t_out;
    t_out.noalias() = R * t_home + t;
    Eigen::Matrix3<Scalar> R_out;
    R_out.noalias() = R * R_home;
    return std::tuple{R_out, t_out, intermediates};
}

// Sanity check: fk_matrix_native must agree with cartan::forward_kinematics.
template <typename Chain>
void verify_fk_matrix_native(const Chain& chain, const std::string& robot)
{
    std::mt19937 rng(123);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto cartan_fk = cartan::forward_kinematics(chain, q);
    auto [R_m, t_m] = fk_matrix_native(chain, q);
    auto R_q = cartan_fk.end_effector.rotation().matrix();
    auto t_q = cartan_fk.end_effector.translation();
    double r_err = (R_m - R_q).norm();
    double t_err = (t_m - t_q).norm();
    if (r_err > 1e-12 || t_err > 1e-12)
        throw std::runtime_error("fk_matrix_native mismatch on " + robot
            + " r=" + std::to_string(r_err) + " t=" + std::to_string(t_err));
}

// Sanity check: fk_matrix_accum must agree with cartan::forward_kinematics.
template <typename Chain>
void verify_fk_matrix_accum(const Chain& chain, const std::string& robot)
{
    std::mt19937 rng(123);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto cartan_fk = cartan::forward_kinematics(chain, q);
    auto [R_m, t_m] = fk_matrix_accum(chain, q);
    auto R_q = cartan_fk.end_effector.rotation().matrix();
    auto t_q = cartan_fk.end_effector.translation();
    double r_err = (R_m - R_q).norm();
    double t_err = (t_m - t_q).norm();
    if (r_err > 1e-12 || t_err > 1e-12)
        throw std::runtime_error("fk_matrix_accum mismatch on " + robot
            + " r=" + std::to_string(r_err) + " t=" + std::to_string(t_err));
}

// Sanity check: fk_matrix_native_full must agree with cartan::forward_kinematics.
template <typename Chain>
void verify_fk_matrix_native_full(const Chain& chain, const std::string& robot)
{
    std::mt19937 rng(123);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto cartan_fk = cartan::forward_kinematics(chain, q);
    auto [R_m, t_m, intermediates] = fk_matrix_native_full(chain, q);
    static_cast<void>(intermediates);
    auto R_q = cartan_fk.end_effector.rotation().matrix();
    auto t_q = cartan_fk.end_effector.translation();
    double r_err = (R_m - R_q).norm();
    double t_err = (t_m - t_q).norm();
    if (r_err > 1e-12 || t_err > 1e-12)
        throw std::runtime_error("fk_matrix_native_full mismatch on " + robot
            + " r=" + std::to_string(r_err) + " t=" + std::to_string(t_err));
}

// Matrix-form space Jacobian. Takes intermediates as pair<Matrix3, Vector3>
// (output of fk_matrix_native_full). Skips the quaternion->matrix conversion
// that jacobian_column does today on every column.
template <typename Scalar, int N, typename Intermediates>
auto space_jacobian_matrix(const cartan::kinematic_chain<Scalar, N>& chain,
                            const Intermediates& intermediates)
{
    using JMat = std::conditional_t<N == cartan::dynamic,
        Eigen::Matrix<Scalar, 6, Eigen::Dynamic>,
        Eigen::Matrix<Scalar, 6, N>>;
    int n = chain.num_joints();
    JMat J;
    if constexpr (N == cartan::dynamic) J.resize(6, n);

    // Column 0: Ad_I * S_0 = S_0
    {
        const auto& s0 = chain.axis(0);
        J.col(0).template head<3>() = s0.omega();
        J.col(0).template tail<3>() = s0.v();
    }

    for (int i = 1; i < n; ++i) {
        const auto& [R, p] = intermediates[static_cast<std::size_t>(i - 1)];
        const auto& s = chain.axis(i);
        auto kind = chain.kind(i);
        switch (kind) {
            case cartan::joint_kind::revolute_x:
            case cartan::joint_kind::revolute_y:
            case cartan::joint_kind::revolute_z: {
                Scalar a = (kind == cartan::joint_kind::revolute_x) ? s.omega()(0)
                         : (kind == cartan::joint_kind::revolute_y) ? s.omega()(1)
                                                                    : s.omega()(2);
                int col_idx = (kind == cartan::joint_kind::revolute_x) ? 0
                            : (kind == cartan::joint_kind::revolute_y) ? 1 : 2;
                Eigen::Vector3<Scalar> R_omega = a * R.col(col_idx);
                Eigen::Vector3<Scalar> R_v;
                R_v.noalias() = R * s.v();
                J.col(i).template head<3>() = R_omega;
                J.col(i).template tail<3>() = p.cross(R_omega) + R_v;
                break;
            }
            default: {
                // generic: full Ad multiplication
                Eigen::Vector3<Scalar> R_omega;
                R_omega.noalias() = R * s.omega();
                Eigen::Vector3<Scalar> R_v;
                R_v.noalias() = R * s.v();
                J.col(i).template head<3>() = R_omega;
                J.col(i).template tail<3>() = p.cross(R_omega) + R_v;
            }
        }
    }
    return J;
}

template <typename Chain>
void verify_jacobian_matrix(const Chain& chain, const std::string& robot)
{
    std::mt19937 rng(123);
    auto q = cartan::fixtures::random_joint_config(chain, rng);
    auto fk_q = cartan::forward_kinematics(chain, q);
    auto J_q = cartan::space_jacobian(chain, fk_q);

    auto [R, t, intermediates] = fk_matrix_native_full(chain, q);
    auto J_m = space_jacobian_matrix(chain, intermediates);
    double err = (J_q - J_m).norm();
    if (err > 1e-10)
        throw std::runtime_error("jacobian matrix mismatch on " + robot
            + " err=" + std::to_string(err));
}

#define FK_BENCH_CARTAN_MATRIX(ROBOT, KC_FACTORY)                                       \
static void bm_fk_##ROBOT##_cartan_matrix(benchmark::State& state)                      \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    verify_fk_matrix_accum(chain, #ROBOT);                                              \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);          \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto r = fk_matrix_accum(chain, q);                                             \
        benchmark::DoNotOptimize(r);                                                    \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_cartan_matrix);                                               \
static void bm_fk_##ROBOT##_cartan_matrix_native(benchmark::State& state)               \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    verify_fk_matrix_native(chain, #ROBOT);                                             \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);          \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto r = fk_matrix_native(chain, q);                                            \
        benchmark::DoNotOptimize(r);                                                    \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_cartan_matrix_native);                                        \
static void bm_fk_##ROBOT##_cartan_matrix_native_full(benchmark::State& state)          \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    verify_fk_matrix_native_full(chain, #ROBOT);                                        \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);          \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto r = fk_matrix_native_full(chain, q);                                       \
        benchmark::DoNotOptimize(r);                                                    \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_cartan_matrix_native_full)

#define FK_BENCH_PINOCCHIO(ROBOT, SC_FACTORY)                                           \
static void bm_fk_##ROBOT##_pinocchio(benchmark::State& state)                          \
{                                                                                       \
    auto sc = SC_FACTORY<double>();                                                     \
    auto pc = build_pinocchio_chain(sc, #ROBOT);                                        \
    verify_equivalence(sc, pc, #ROBOT);                                                 \
    std::mt19937 rng(42);                                                               \
    auto q_cartan = random_config_static(sc, rng);                                      \
    Eigen::VectorXd q = cartan_q_to_pinocchio(sc, q_cartan);                            \
    for (auto _ : state)                                                                \
    {                                                                                   \
        pinocchio::framesForwardKinematics(pc.model, pc.data, q);                       \
        benchmark::DoNotOptimize(pc.data.oMf[pc.ee_frame_id]);                          \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_pinocchio)

#define FK_BENCH_CARTAN_STATIC(ROBOT, SC_FACTORY)                                       \
static void bm_fk_##ROBOT##_cartan_static(benchmark::State& state)                      \
{                                                                                       \
    auto sc = SC_FACTORY<double>();                                                     \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(random_config_static(sc, rng)), kInputs> qs;                    \
    for (auto& q : qs) q = random_config_static(sc, rng);                               \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto result = cartan::forward_kinematics(sc, q);                                \
        benchmark::DoNotOptimize(result);                                               \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_cartan_static)

#define FK_BENCH_CARTAN_KC(ROBOT, KC_FACTORY)                                           \
static void bm_fk_##ROBOT##_cartan_kc(benchmark::State& state)                          \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);          \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto result = cartan::forward_kinematics(chain, q);                             \
        benchmark::DoNotOptimize(result);                                               \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_cartan_kc)

#define JAC_BENCH_CARTAN(ROBOT, KC_FACTORY)                                             \
static void bm_jac_##ROBOT##_cartan(benchmark::State& state)                            \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::forward_kinematics(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fks; \
    for (auto& f : fks) f = cartan::forward_kinematics(chain, cartan::fixtures::random_joint_config(chain, rng)); \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& fk = fks[i++ & (kInputs - 1)];                                           \
        benchmark::DoNotOptimize(fk);                                                   \
        auto J = cartan::space_jacobian(chain, fk);                                     \
        benchmark::DoNotOptimize(J);                                                    \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_jac_##ROBOT##_cartan);                                                     \
static void bm_jac_##ROBOT##_matrix(benchmark::State& state)                            \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::forward_kinematics_matrix(chain, cartan::fixtures::random_joint_config(chain, rng))), kInputs> fkms; \
    for (auto& f : fkms) f = cartan::forward_kinematics_matrix(chain, cartan::fixtures::random_joint_config(chain, rng)); \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& fkm = fkms[i++ & (kInputs - 1)];                                         \
        benchmark::DoNotOptimize(fkm);                                                  \
        auto J = cartan::space_jacobian(chain, fkm);                                    \
        benchmark::DoNotOptimize(J);                                                    \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_jac_##ROBOT##_matrix);                                                     \
static void bm_fk_##ROBOT##_matrix_api(benchmark::State& state)                         \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);          \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto fkm = cartan::forward_kinematics_matrix(chain, q);                         \
        benchmark::DoNotOptimize(fkm);                                                  \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_matrix_api);                                                  \
static void bm_fkjac_##ROBOT##_cartan(benchmark::State& state)                          \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);          \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto fk = cartan::forward_kinematics(chain, q);                                 \
        auto J = cartan::space_jacobian(chain, fk);                                     \
        benchmark::DoNotOptimize(J);                                                    \
        benchmark::DoNotOptimize(fk);                                                   \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fkjac_##ROBOT##_cartan);                                                   \
static void bm_fkjac_##ROBOT##_matrix(benchmark::State& state)                          \
{                                                                                       \
    auto chain = cartan::fixtures::KC_FACTORY<double>();                              \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(cartan::fixtures::random_joint_config(chain, rng)), kInputs> qs; \
    for (auto& q : qs) q = cartan::fixtures::random_joint_config(chain, rng);          \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto fkm = cartan::forward_kinematics_matrix(chain, q);                         \
        auto J = cartan::space_jacobian(chain, fkm);                                    \
        benchmark::DoNotOptimize(J);                                                    \
        benchmark::DoNotOptimize(fkm);                                                  \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fkjac_##ROBOT##_matrix)

#define FK_BENCH_CARTAN_STATIC_MATRIX(ROBOT, SC_FACTORY)                                \
static void bm_fk_##ROBOT##_matrix_static(benchmark::State& state)                      \
{                                                                                       \
    auto sc = SC_FACTORY<double>();                                                     \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(random_config_static(sc, rng)), kInputs> qs;                    \
    for (auto& q : qs) q = random_config_static(sc, rng);                               \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto fkm = cartan::forward_kinematics_matrix(sc, q);                            \
        benchmark::DoNotOptimize(fkm);                                                  \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fk_##ROBOT##_matrix_static);                                               \
static void bm_fkjac_##ROBOT##_matrix_static(benchmark::State& state)                   \
{                                                                                       \
    auto sc = SC_FACTORY<double>();                                                     \
    std::mt19937 rng(42);                                                               \
    std::array<decltype(random_config_static(sc, rng)), kInputs> qs;                    \
    for (auto& q : qs) q = random_config_static(sc, rng);                               \
    std::size_t i = 0;                                                                  \
    for (auto _ : state)                                                                \
    {                                                                                   \
        auto& q = qs[i++ & (kInputs - 1)];                                             \
        benchmark::DoNotOptimize(q);                                                    \
        auto fkm = cartan::forward_kinematics_matrix(sc, q);                            \
        auto J = cartan::space_jacobian(sc, fkm);                                       \
        benchmark::DoNotOptimize(J);                                                    \
        benchmark::DoNotOptimize(fkm);                                                  \
    }                                                                                   \
}                                                                                       \
BENCHMARK(bm_fkjac_##ROBOT##_matrix_static)

#define FK_PINOCCHIO_COMPARISON(ROBOT, KC_FACTORY, SC_FACTORY)                          \
    FK_BENCH_CARTAN_KC(ROBOT, KC_FACTORY);                                              \
    FK_BENCH_CARTAN_STATIC(ROBOT, SC_FACTORY);                                          \
    FK_BENCH_CARTAN_MATRIX(ROBOT, KC_FACTORY);                                          \
    FK_BENCH_PINOCCHIO(ROBOT, SC_FACTORY);                                              \
    JAC_BENCH_CARTAN(ROBOT, KC_FACTORY);                                                \
    FK_BENCH_CARTAN_STATIC_MATRIX(ROBOT, SC_FACTORY)

FK_PINOCCHIO_COMPARISON(planar_3r,  make_3r_planar_chain,  make_3r_planar_static);
FK_PINOCCHIO_COMPARISON(ur3e,       make_ur3e_chain,       make_ur3e_static);
FK_PINOCCHIO_COMPARISON(lbr_med14,  make_lbr_med14_chain,  make_lbr_med14_static);
FK_PINOCCHIO_COMPARISON(kr6_sixx,   make_kr6_sixx_chain,   make_kr6_sixx_static);
FK_PINOCCHIO_COMPARISON(panda,      make_panda_chain,      make_panda_static);
FK_PINOCCHIO_COMPARISON(abb_irb120, make_abb_irb120_chain, make_abb_irb120_static);
FK_PINOCCHIO_COMPARISON(jaco2,      make_jaco2_chain,      make_jaco2_static);
FK_PINOCCHIO_COMPARISON(fetch,      make_fetch_chain,      make_fetch_static);
FK_PINOCCHIO_COMPARISON(baxter,     make_baxter_chain,     make_baxter_static);

}
