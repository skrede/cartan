#ifndef HPP_GUARD_CARTAN_SERIAL_FK_FORWARD_KINEMATICS_MATRIX_H
#define HPP_GUARD_CARTAN_SERIAL_FK_FORWARD_KINEMATICS_MATRIX_H

/// Matrix-form Product of Exponentials FK.
///
/// Stores rotation as a 3x3 matrix in the per-joint cumulative intermediates,
/// avoiding the quaternion product in compose and the quaternion->matrix
/// conversion that downstream Jacobian computation would otherwise pay on
/// every column. Empirically ~30-40% faster than the quaternion-form
/// `forward_kinematics` on 6/7-DOF chains; the gain comes from SIMD/ILP
/// friendliness of 3x3 matmul plus avoidance of the quat->mat conversion
/// in `jacobian_column`.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138.

#include "cartan/types.h"

#include "cartan/serial/fk/detail/shape_validation.h"
#include "cartan/serial/fk/detail/axis_specializations.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/chain/static_chain.h"

#include <array>
#include <utility>

namespace cartan
{

/// Per-joint cumulative pose stored as (R, p) in matrix form.
template <typename Scalar>
struct pose_matrix
{
    matrix3<Scalar> R{matrix3<Scalar>::Identity()};
    vector3<Scalar> p{vector3<Scalar>::Zero()};
};

/// Result of matrix-form forward kinematics. Mirrors `fk_result` but with
/// matrix-form rotation in the per-joint intermediates and end-effector pose.
template <typename Scalar = double, int N = dynamic>
struct fk_matrix_result
{
    static_assert(std::is_floating_point_v<Scalar>,
        "fk_matrix_result requires a floating-point Scalar type");

    using intermediate_storage = std::conditional_t<
        N == dynamic,
        std::vector<pose_matrix<Scalar>>,
        std::array<pose_matrix<Scalar>, static_cast<std::size_t>((N == dynamic) ? 0 : N)>>;

    pose_matrix<Scalar> end_effector{};
    intermediate_storage intermediates{};

    int num_joints() const
    {
        return static_cast<int>(intermediates.size());
    }
};

/// Matrix-form forward kinematics for a caller that has already established
/// that q holds exactly chain.num_joints() finite components. Neither
/// precondition is checked here, and violating either is undefined behavior on
/// the same terms as `forward_kinematics_unchecked`.
template <typename Scalar, int N>
fk_matrix_result<Scalar, N> forward_kinematics_matrix_unchecked(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q)
{
    fk_matrix_result<Scalar, N> result;
    int n = chain.num_joints();

    if constexpr (N == dynamic)
    {
        result.intermediates.resize(static_cast<std::size_t>(n));
    }

    matrix3<Scalar> R = matrix3<Scalar>::Identity();
    vector3<Scalar> p = vector3<Scalar>::Zero();
    matrix3<Scalar> R_step;
    vector3<Scalar> t_step;

    for (int i = 0; i < n; ++i)
    {
        switch (chain.kind(i))
        {
            case joint_kind::revolute_x:  detail::exp_joint_matrix<revolute_x>(q(i), chain.axis(i), R_step, t_step); break;
            case joint_kind::revolute_y:  detail::exp_joint_matrix<revolute_y>(q(i), chain.axis(i), R_step, t_step); break;
            case joint_kind::revolute_z:  detail::exp_joint_matrix<revolute_z>(q(i), chain.axis(i), R_step, t_step); break;
            case joint_kind::prismatic_x: detail::exp_joint_matrix<prismatic_x>(q(i), chain.axis(i), R_step, t_step); break;
            case joint_kind::prismatic_y: detail::exp_joint_matrix<prismatic_y>(q(i), chain.axis(i), R_step, t_step); break;
            case joint_kind::prismatic_z: detail::exp_joint_matrix<prismatic_z>(q(i), chain.axis(i), R_step, t_step); break;
            case joint_kind::general:
            default:
            {
                se3<Scalar> step = detail::exp_joint_runtime(chain.kind(i), q(i), chain.axis(i));
                R_step = step.rotation().matrix();
                t_step = step.translation();
                break;
            }
        }
        vector3<Scalar> p_new;
        p_new.noalias() = R * t_step + p;
        matrix3<Scalar> R_new;
        R_new.noalias() = R * R_step;
        p = p_new;
        R = R_new;
        result.intermediates[static_cast<std::size_t>(i)] = pose_matrix<Scalar>{R, p};
    }

    auto R_home = chain.home().rotation().matrix();
    auto t_home = chain.home().translation();
    result.end_effector.p.noalias() = R * t_home + p;
    result.end_effector.R.noalias() = R * R_home;
    return result;
}

/// Matrix-form forward kinematics for a kinematic chain.
///
/// q is taken at the caller's own type for the reason spelled out over
/// `forward_kinematics`: a position_type parameter converts an ill-sized vector
/// in the caller's frame, where no predicate here can see the over-read.
template <typename Scalar, int N, typename Derived>
cartan::expected<fk_matrix_result<Scalar, N>, chain_failure> forward_kinematics_matrix(
    const kinematic_chain<Scalar, N>& chain,
    const Eigen::MatrixBase<Derived>& q)
{
    return detail::guarded(detail::check_joint_positions(chain, q),
        [&] { return forward_kinematics_matrix_unchecked(chain, q.derived()); });
}

/// Matrix-form forward kinematics for a static_chain under the same unchecked
/// precondition on q. Joint tags are known at compile time; dispatches into
/// per-tag `exp_joint_matrix`.
template <typename Scalar, joint_tag... Joints>
fk_matrix_result<Scalar, static_cast<int>(sizeof...(Joints))>
forward_kinematics_matrix_unchecked(
    const static_chain<Scalar, Joints...>& chain,
    const typename joint_state<Scalar, static_cast<int>(sizeof...(Joints))>::position_type& q)
{
    constexpr int N = static_cast<int>(sizeof...(Joints));
    fk_matrix_result<Scalar, N> result;

    matrix3<Scalar> R = matrix3<Scalar>::Identity();
    vector3<Scalar> p = vector3<Scalar>::Zero();

    [&]<std::size_t... Is>(std::index_sequence<Is...>)
    {
        using joint_tuple = std::tuple<Joints...>;
        ((
            [&]<std::size_t I>()
            {
                using Joint = std::tuple_element_t<I, joint_tuple>;
                matrix3<Scalar> R_step;
                vector3<Scalar> t_step;
                detail::exp_joint_matrix<Joint>(
                    q(static_cast<int>(I)),
                    chain.axis(static_cast<int>(I)),
                    R_step, t_step);
                vector3<Scalar> p_new;
                p_new.noalias() = R * t_step + p;
                matrix3<Scalar> R_new;
                R_new.noalias() = R * R_step;
                p = p_new;
                R = R_new;
                result.intermediates[I] = pose_matrix<Scalar>{R, p};
            }.template operator()<Is>()
        ), ...);
    }(std::make_index_sequence<static_cast<std::size_t>(N)>{});

    auto R_home = chain.home().rotation().matrix();
    auto t_home = chain.home().translation();
    result.end_effector.p.noalias() = R * t_home + p;
    result.end_effector.R.noalias() = R * R_home;
    return result;
}

/// Matrix-form forward kinematics for a static_chain.
template <typename Scalar, joint_tag... Joints, typename Derived>
cartan::expected<fk_matrix_result<Scalar, static_cast<int>(sizeof...(Joints))>, chain_failure>
forward_kinematics_matrix(
    const static_chain<Scalar, Joints...>& chain,
    const Eigen::MatrixBase<Derived>& q)
{
    return detail::guarded(detail::check_joint_positions(chain, q),
        [&] { return forward_kinematics_matrix_unchecked(chain, q.derived()); });
}

}

#endif
