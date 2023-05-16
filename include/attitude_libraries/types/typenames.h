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

/**
 * The base structure type for all of the param structures
**/
template <typename Scalar>
struct BaseParams
{
    Scalar dt = 0;
};

/**
 * The base structure type for all of the filter data structures
**/
template <typename Scalar>
struct FilterData
{
    Quaternion<Scalar> quat = Quaternion<Scalar>::Zero();
};

/**
 * The base structure type for all of the controller data structures
**/
template <typename Scalar>
struct ControllerData
{
    // Controller state variables 
    Quaternion<Scalar> quat = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> omega = BodyRate<Scalar>::Zero();

    // Controller desired variables
    Quaternion<Scalar> quatDesired = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> omegaDesired = BodyRate<Scalar>::Zero();
    BodyRate<Scalar> omegaDotDesired = BodyRate<Scalar>::Zero();

    Control<Scalar> u = Control<Scalar>::Zero();
};

} // namespace attitude

#endif // TYPENAME_H_
