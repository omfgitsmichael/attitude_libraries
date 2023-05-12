#ifndef TYPENAME_H_
#define TYPENAME_H_

#include <optional>

#include <eigen3/Eigen/Dense>

namespace attitude {

/**
 * Quaternion typename
**/
template <typename Scalar>
using Quaternion = Eigen::Vector<Scalar, 4>;

/**
 * Optional quaternion typename
**/
template <typename Scalar>
using OptionalQuaternion = std::optional<Eigen::Vector<Scalar, 4>>;

/**
 * Euler angle typename
**/
template <typename Scalar>
using EulerAngle = Eigen::Vector<Scalar, 3>;

/**
 * Optional euler angle typename
**/
template <typename Scalar>
using OptionalEulerAngle = std::optional<Eigen::Vector<Scalar, 3>>;

/**
 * Body rate typename
**/
template <typename Scalar>
using BodyRate = Eigen::Vector<Scalar, 3>;

/**
 * Optional body rate typename
**/
template <typename Scalar>
using OptionalBodyRate = std::optional<Eigen::Vector<Scalar, 3>>;

/**
 * Rotation matrix typename
**/
template <typename Scalar>
using RotationMatrix = Eigen::Matrix<Scalar, 3, 3>;

/**
 * Optional rotation matrix typename
**/
template <typename Scalar>
using OptionalRotationMatrix = std::optional<Eigen::Matrix<Scalar, 3, 3>>;

/**
 * System control output typename
**/
template <typename Scalar>
using Control = Eigen::Vector<Scalar, 3>;

} // namespace attitude

#endif // TYPENAME_H_
