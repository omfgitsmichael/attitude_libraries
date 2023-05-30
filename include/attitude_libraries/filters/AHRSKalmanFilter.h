#ifndef AHRS_KALMAN_FILTER_H_
#define AHRS_KALMAN_FILTER_H_

#include "attitude_libraries/quaternions/quaternionMath.h"
#include "attitude_libraries/types/filterTypes.h"
#include "attitude_libraries/utils/attitudeUtils.h"

namespace attitude {

namespace filter {

namespace ahrs {

/**
* Initializes the Kalman Filter state estimates using accelerometer and magnetometer measurements forming an E-compass. Referenced
* from Mathworks: https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: accelerometer - The accelerometer measurement in the body frame referenced from NED frame
* Input: magnetometer - The magnetometer measurement in the body frame referenced from NED frame
* Input: omega - The bodys measured angular rates
* Output: Boolean if it passed or failed
**/
template <typename Scalar>
inline bool AHRSKalmanInitialize(AHRSData<Scalar>& data)
{
    AttitudeMeasurement<Scalar> accelerometerMeas;
    AttitudeMeasurement<Scalar> magnetometerMeas;
    for (const auto meas : data.attitudeMeasurements) {
        if (meas.valid && meas.sensorType == AttitudeMeasurementType::ACCELEROMETER) {
            accelerometerMeas = meas;
        } else if (meas.valid && meas.sensorType == AttitudeMeasurementType::MAGNETOMETER) {
            magnetometerMeas = meas;
        } else {
            return false;
        }
    }

    const Eigen::Vector<Scalar, 3> cross1 = accelerometerMeas.attitudeMeasVector.cross(magnetometerMeas.attitudeMeasVector);
    const Eigen::Vector<Scalar, 3> cross2 = cross1.cross(accelerometerMeas.attitudeMeasVector);

    RotationMatrix<Scalar> rotation = RotationMatrix<Scalar>::Zero();
    rotation.col(0) = cross2 / cross2.norm();
    rotation.col(1) = cross1 / cross1.norm();
    rotation.col(2) = accelerometerMeas.attitudeMeasVector / accelerometerMeas.attitudeMeasVector.norm();

    OptionalQuaternion<Scalar> quat = rotationToQuaternion(rotation);

    if (!quat) {
        return false;
    }

    data.omega = data.omegaMeas;
    data.quat = *quat;
    data.magneticVector = rotation.transpose() * magnetometerMeas.attitudeMeasVector;
    return true;
}

/**
* Propagates the state estimates of the Kalman Filter to 'current time' using a discrete time formulation for the quaternion
* and process noise. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: params - AHRS params
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline void AHRSKalmanPropagate(const AHRSParams<Scalar>& params, AHRSData<Scalar>& data)
{
    // First "propagate" the angular velocity
    data.omega = data.omegaMeas - data.omegaBias;

    const Scalar dt = params.dt;
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();
    const Scalar w = data.omega.norm();

    // Discrete time quaternion propagation
    Eigen::Matrix<Scalar, 4, 4> omega = Eigen::Matrix<Scalar, 4, 4>::Identity();

    if (w != static_cast<Scalar>(0.0)){
        const Scalar scalarTerm = static_cast<Scalar>(std::cos(0.5 * w * dt));
        const BodyRate<Scalar> psi = (static_cast<Scalar>(std::sin(0.5 * w * dt)) / w) * data.omega;
        const Eigen::Matrix<Scalar, 3, 3> phiCross{{0.0, -psi(2), psi(1)},
                                                {psi(2), 0.0, -psi(0)},
                                                {-psi(1), psi(0), 0.0}};
        
        omega.block(0, 0, 3, 3) = scalarTerm * identity - phiCross;
        omega.block(0, 3, 3, 1) = psi;
        omega.block(3, 0, 1, 3) = -psi.transpose();
        omega(3, 3) = scalarTerm;
    }

    // Propagate the covariance matrix -- pretty sure this can be re-written as FPF' + Q
    const Scalar gyroNoise = params.omegaProcessNoise * params.omegaProcessNoise;
    const Eigen::Matrix<Scalar, 3, 3> QGyroNoise = gyroNoise * identity;

    const Scalar gyroBiasProcess = params.biasProcessNoise * params.biasProcessNoise;
    const Eigen::Matrix<Scalar, 3, 3> QGyroBiasProcess = gyroBiasProcess * identity;

    const Scalar linearAccelProcess = params.linearAccelNoise * params.linearAccelNoise;
    const Eigen::Matrix<Scalar, 3, 3> QLinearAccelProcess = linearAccelProcess * identity;

    const Scalar magneticDisturbanceProcess = params.magDisturbanceNoise * params.magDisturbanceNoise;
    const Eigen::Matrix<Scalar, 3, 3> QMagneticDisturbanceProcess = magneticDisturbanceProcess * identity;

    Covariance<Scalar, 12> process = Covariance<Scalar, 12>::Zero();
    process.block(0, 0, 3, 3) = data.P.block(0, 0, 3, 3) + dt * dt * (data.P.block(3, 3, 3, 3) + QGyroNoise + QGyroBiasProcess);
    process.block(0, 3, 3, 3) = -dt * (data.P.block(0, 3, 3, 3) + QGyroBiasProcess);
    process.block(3, 0, 3, 3) = process.block(0, 3, 3, 3);
    process.block(3, 3, 3, 3) = data.P.block(3, 3, 3, 3) + QGyroBiasProcess;
    process.block(6, 6, 3, 3) = static_cast<Scalar>(0.25) * data.P.block(6, 6, 3, 3) + QLinearAccelProcess;
    process.block(9, 9, 3, 3) = static_cast<Scalar>(0.25) * data.P.block(9, 9, 3, 3) + QMagneticDisturbanceProcess;

    data.quat = omega * data.quat;
    data.P = process;
}

/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm using
* accelerometer measurements -- variant of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: measurement - Accelerometer measurement
* Input: params - AHRS params
* Input: rotation - Rotation matrix
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline void accelerometerUpdate(const AttitudeMeasurement<Scalar>& measurement,
                                const AHRSParams<Scalar>& params,
                                const RotationMatrix<Scalar>& rotation,
                                AHRSData<Scalar>& data)
{
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();

    // Measurement and error model
    const AttitudeVector<Scalar> estMeas = params.gravity * rotation.col(2) + data.linearAccelForces;
    const AttitudeVector<Scalar> meas = measurement.attitudeMeasVector;
    const AttitudeVector<Scalar> residual = meas - estMeas;

    // Calculate the measurement sensitivity matrix
    const Eigen::Matrix<Scalar, 3, 3> measCross{{0.0, -estMeas(2), estMeas(1)},
                                                {estMeas(2), 0.0, -estMeas(0)},
                                                {-estMeas(1), estMeas(0), 0.0}};
    Eigen::Matrix<Scalar, 3, 12> H = Eigen::Matrix<Scalar, 3, 12>::Zero();
    H.block(0, 0, 3, 3) = measCross;
    H.block(0, 6, 3, 3) = identity;

    // Calculate the Kalman gain
    const Scalar measurementNoise = measurement.sigma * measurement.sigma +
        params.linearAccelNoise * params.linearAccelNoise + params.dt * params.dt * (params.biasProcessNoise * params.biasProcessNoise +
        params.omegaProcessNoise * params.omegaProcessNoise);
    const Eigen::Matrix<Scalar, 3, 3> R = measurementNoise * identity;
    Eigen::Matrix<Scalar, 3, 3> S = H * data.P * H.transpose() + R;
    Eigen::Matrix<Scalar, 12, 3> K = data.P * H.transpose() * S.inverse();

    // Update states and costates
    data.deltaX += K * (residual - H * data.deltaX);
    data.P = (Eigen::Matrix<Scalar, 12, 12>::Identity() - K * H) * data.P;
}

/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm using
* magnetometer measurements -- variant of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: measurement - Magnetometer measurement
* Input: params - AHRS params
* Input: rotation - Rotation matrix
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline bool magnetometerUpdate(const AttitudeMeasurement<Scalar>& measurement,
                               const AHRSParams<Scalar>& params,
                               const RotationMatrix<Scalar>& rotation,
                               AHRSData<Scalar>& data)
{
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();

    // Measurement and error model
    const AttitudeVector<Scalar> estMeas = rotation * data.magneticVector - data.magneticDisturbances;
    const AttitudeVector<Scalar> meas = measurement.attitudeMeasVector;
    const AttitudeVector<Scalar> residual = meas - estMeas;

    // Calculate the measurement sensitivity matrix
    const Eigen::Matrix<Scalar, 3, 3> measCross{{0.0, -estMeas(2), estMeas(1)},
                                                {estMeas(2), 0.0, -estMeas(0)},
                                                {-estMeas(1), estMeas(0), 0.0}};
    Eigen::Matrix<Scalar, 3, 12> H = Eigen::Matrix<Scalar, 3, 12>::Zero();
    H.block(0, 0, 3, 3) = measCross;
    H.block(0, 9, 3, 3) = -identity;

    // Calculate the Kalman gain
    const Scalar measurementNoise = measurement.sigma * measurement.sigma +
        params.magDisturbanceNoise * params.magDisturbanceNoise + params.dt * params.dt * (params.biasProcessNoise * params.biasProcessNoise +
        params.omegaProcessNoise * params.omegaProcessNoise);
    const Eigen::Matrix<Scalar, 3, 3> R = measurementNoise * identity;
    Eigen::Matrix<Scalar, 3, 3> S = H * data.P * H.transpose() + R;
    Eigen::Matrix<Scalar, 12, 3> K = data.P * H.transpose() * S.inverse();

    // Update states and costates
    data.deltaX += K * (residual - H * data.deltaX);
    data.P = (Eigen::Matrix<Scalar, 12, 12>::Identity() - K * H) * data.P;

    return true;
}

/**
* Update the estimated states for the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm -- variant
* of MEKF. Referenced from Mathworks:
* https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: params - AHRS params
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline bool AHRSKalmanUpdate(const AHRSParams<Scalar>& params, AHRSData<Scalar>& data)
{
    const OptionalRotationMatrix<Scalar> rotation = quaternionRotationMatrix(data.quat);
    if (!rotation){
        return false;
    }

    data.deltaX = DeltaStates<Scalar, 12>::Zero();

    // Update the internal states
    bool magnetometerValid = false;
    for (const auto meas : data.attitudeMeasurements) {
        if (meas.valid && meas.sensorType == AttitudeMeasurementType::ACCELEROMETER) {
            accelerometerUpdate(meas, params, *rotation, data);
        } else if (meas.valid && meas.sensorType == AttitudeMeasurementType::MAGNETOMETER) {
            magnetometerUpdate(meas, params, *rotation, data);

            magnetometerValid = true;
        }
    }

    // Update the primary states
    data.omegaBias += data.deltaX.segment(3, 3);
    data.linearAccelForces += data.deltaX.segment(6, 3);
    data.magneticDisturbances += data.deltaX.tail(3);
    data.omega = data.omegaMeas - data.omegaBias;

    const Eigen::Matrix<Scalar, 4, 3> E{{data.quat(3), -data.quat(2), data.quat(1)},
                                        {data.quat(2), data.quat(3), -data.quat(0)},
                                        {-data.quat(1), data.quat(0), data.quat(3)},
                                        {-data.quat(0), -data.quat(1), -data.quat(2)}};
    data.quat += static_cast<Scalar>(0.5) * E * data.deltaX.head(3);
    data.quat /= data.quat.norm();

    // Update the magnetic vector if the magnetometer measurement was valid
    if (magnetometerValid) {
        const AttitudeVector<Scalar> disturbanceNED = (*rotation).transpose() * data.magneticDisturbances;
        const AttitudeVector<Scalar> magneticNED = data.magneticVector - disturbanceNED;
        const Scalar inclination = static_cast<Scalar>(std::atan2(magneticNED(2), magneticNED(0)));
        data.magneticVector = params.geomagneticFieldStrength *  AttitudeVector<Scalar>{std::cos(inclination), 0.0, std::sin(inclination)};
    }

    return true;
}

/**
* Runs the attitude and heading reference systems (AHRS) extended Kalman Filter (MEKF) algorithm -- variant of MEKF. Referenced
* from Mathworks: https://www.mathworks.com/help/fusion/ref/ahrsfilter-system-object.html#mw_a42526cc-0fa3-40b9-936e-343972701689
* Input: params - AHRS params
* Input: data - AHRS data structure
* Output: Boolean if it passsed or failed
**/
template <typename Scalar>
inline bool AHRSKalmanFilter(const AHRSParams<Scalar>& params, AHRSData<Scalar>& data)
{
    bool result = false;
    if (data.initialize) {
        result = AHRSKalmanInitialize(data);

        if (!result) {
            return result;
        }

        data.initialize = false;
    }

    // Propagate the states to 'current time'
    AHRSKalmanPropagate(params, data);

    // Update the states based off the current measurements
    result = AHRSKalmanUpdate(params, data);

    return result;
}

/**
* Reset the AHRS filter specific data.
* Input: data - The AHRS Filter data structure
* Input: value - Gain for the covariance matrix
* Output:
**/
template <typename Scalar>
inline void AHRSKalmanFilterReset(AHRSData<Scalar>& data, Scalar value)
{
    data.linearAccelForces = AttitudeVector<Scalar>::Zero();
    data.magneticDisturbances = AttitudeVector<Scalar>::Zero();
    data.magneticVector = AttitudeVector<Scalar>::Zero();
    data.deltaX = DeltaStates<Scalar, 12>::Zero();
    data.P = Covariance<Scalar, 12>::Identity() * value;
    data.initialize = true;
}

} // namespace ahrs

} // namespace filter

} // namespace attitude

#endif // AHRS_KALMAN_FILTER_H_
