#ifndef FILTER_TYPES_H_
#define FILTER_TYPES_H_

#include "attitude_libraries/types/typenames.h"

namespace attitude {

namespace filter {
/**
 * The delta X update typename
**/
template <typename Scalar, int N>
using DeltaStates = Eigen::Vector<Scalar, N>;

/**
 * The delta X update typename
**/
template <typename Scalar, int N>
using Covariance = Eigen::Matrix<Scalar, N, N>;

namespace mekf {

/**
 * Kalman Filter propagation parameters
**/
template <typename Scalar>
struct MEKFParams : BaseParams<Scalar>
{
    Scalar omegaProcessNoise = 0.0; /// Angular rate process noise standard deviation
    Scalar biasProcessNoise = 0.0;  /// Angular rate bias process noise standard deviation
};

/**
 * Attitude estimation Kalman Filter data type structure
**/
template <typename Scalar>
struct MEKFData : FilterData<Scalar>
{
    // Kalman Filter estimated states
    DeltaStates<Scalar, 6> deltaX = DeltaStates<Scalar, 6>::Zero();

    // Covariance
    Covariance<Scalar, 6> P = Covariance<Scalar, 6>::Zero();
};

} // namespace mekf

namespace ahrs {

/**
 * Kalman Filter propagation parameters
**/
template <typename Scalar>
struct AHRSParams : BaseParams<Scalar>
{
    Scalar omegaProcessNoise = 0.0;   /// Angular rate process noise standard deviation
    Scalar biasProcessNoise = 0.0;    /// Angular rate bias process noise standard deviation
    Scalar linearAccelNoise = 0.0;    /// Linear acceleration force process noise standard deviation
    Scalar magDisturbanceNoise = 0.0; /// Magnetic disturbance process noise standard deviation

    // Calibrated parameters
    Scalar gravity = 9.81;                  /// Defaulted to 9.81 m/s^2
    Scalar geomagneticFieldStrength = 50.0; /// Defaulted to 50 microTesla
};

/**
 * Attitude estimation Kalman Filter data type structure.
**/
template <typename Scalar>
struct AHRSData : FilterData<Scalar>
{
    // Kalman Filter estimated states
    AttitudeVector<Scalar> linearAccelForces = AttitudeVector<Scalar>::Zero();
    AttitudeVector<Scalar> magneticDisturbances = AttitudeVector<Scalar>::Zero();
    AttitudeVector<Scalar> magneticVector = AttitudeVector<Scalar>::Zero();
    DeltaStates<Scalar, 12> deltaX = DeltaStates<Scalar, 12>::Zero();

    // Covariance
    Covariance<Scalar, 12> P = Covariance<Scalar, 12>::Zero();
};

} // namespace ahrs

} // namespace filter

} // namespace attitude

#endif // FILTER_TYPES_H_
