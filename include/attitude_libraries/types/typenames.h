#ifndef TYPENAME_H_
#define TYPENAME_H_

#include <optional>
#include <vector>

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
 * Attitude vector typename
**/
template <typename Scalar>
using AttitudeVector = Eigen::Vector<Scalar, 3>;

/**
 * Attitude measurement sensor type enumerator 
**/
enum AttitudeMeasurementType
{
    STAR_TRACKER = 0,
    MAGNETOMETER = 1,
    ACCELEROMETER = 2,
};

/**
 * Kalman Filter attitude measurement structure
**/
template <typename Scalar>
struct AttitudeMeasurement
{
    bool valid = false;
    Scalar sigma = 0.0; /// Attitude measurement measurement noise standard deviation
    AttitudeVector<Scalar> attitudeRefVector = AttitudeVector<Scalar>::Zero();
    AttitudeVector<Scalar> attitudeMeasVector = AttitudeVector<Scalar>::Zero();
    AttitudeMeasurementType sensorType = AttitudeMeasurementType::STAR_TRACKER;
};

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
    // Measurements
    std::vector<AttitudeMeasurement<Scalar>> attitudeMeasurements; /// Size is the number of incoming measurement vectors

    // Measurements
    BodyRate<Scalar> omegaMeas = BodyRate<Scalar>::Zero();

    // Estimated data
    Quaternion<Scalar> quat = Quaternion<Scalar>::Zero();
    BodyRate<Scalar> omega = BodyRate<Scalar>::Zero();
    BodyRate<Scalar> omegaBias = BodyRate<Scalar>::Zero();

    // Initialization boolean
    bool initialize = true;
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
