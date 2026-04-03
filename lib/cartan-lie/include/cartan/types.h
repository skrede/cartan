#ifndef HPP_GUARD_CARTAN_TYPES_H
#define HPP_GUARD_CARTAN_TYPES_H

#include <Eigen/Dense>

#include <cstddef>
#include <numbers>

namespace cartan
{

template <typename Scalar, std::size_t Rows, std::size_t Cols>
using matrix = Eigen::Matrix<Scalar, static_cast<int>(Rows), static_cast<int>(Cols)>;

template <typename Scalar, std::size_t N>
using vector = Eigen::Matrix<Scalar, static_cast<int>(N), 1>;

template <typename Scalar>
using vector2 = Eigen::Matrix<Scalar, 2, 1>;

template <typename Scalar>
using vector3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using vector6 = Eigen::Matrix<Scalar, 6, 1>;

template <typename Scalar>
using matrix2 = Eigen::Matrix<Scalar, 2, 2>;

template <typename Scalar>
using matrix3 = Eigen::Matrix<Scalar, 3, 3>;

template <typename Scalar>
using matrix4 = Eigen::Matrix<Scalar, 4, 4>;

template <typename Scalar>
using matrix6 = Eigen::Matrix<Scalar, 6, 6>;

template <typename Scalar>
using quaternion = Eigen::Quaternion<Scalar>;

/// Sentinel value indicating dynamic (runtime-determined) size.
inline constexpr int dynamic = -1;

}

#endif
