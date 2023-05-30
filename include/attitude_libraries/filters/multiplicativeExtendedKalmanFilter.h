#ifndef ATTITUDE_KALMAN_FILTER_H_
#define ATTITUDE_KALMAN_FILTER_H_

#include "attitude_libraries/quaternions/quaternionMath.h"
#include "attitude_libraries/types/filterTypes.h"
#include "attitude_libraries/utils/attitudeUtils.h"

namespace attitude {

namespace filter {

namespace mekf {

/**
* Initializes the Kalman Filter state estimates. Referenced from "Optimal Estimation of Dynamic Systems" by John L. Crassidis and 
* John L. Junkins, 2nd edition.
* Input:
* Output:
**/
template <typename Scalar>
inline void multiplicativeExtendedKalmanInitialize()
{
}

/**
* Propagates the state estimates of the Kalman Filter to 'current time' using a discrete time formulation for the quaternion
* and process noise. Referenced from "Optimal Estimation of Dynamic Systems" by John L. Crassidis and John L. Junkins, 2nd edition.
* Input: params - MEKF params
* Input: data - MEKF data structure
**/
template <typename Scalar>
inline void multiplicativeExtendedKalmanPropagate(const MEKFParams<Scalar>& params, MEKFData<Scalar>& data)
{
    // First "propagate" the angular velocity
    data.omega = data.omegaMeas - data.omegaBias;

    const Scalar dt = params.dt;
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity();
    const Scalar w = data.omega.norm();
    const Scalar w2 = w * w;
    const Scalar w3 = w * w * w;

    // Discrete time propagation
    Eigen::Matrix<Scalar, 4, 4> omega = Eigen::Matrix<Scalar, 4, 4>::Identity();
    Eigen::Matrix<Scalar, 6, 6> phi = Eigen::Matrix<Scalar, 6, 6>::Identity();
    phi.block(0, 3, 3, 3) = -identity * dt;
    Eigen::Matrix<Scalar, 6, 6> gamma = Eigen::Matrix<Scalar, 6, 6>::Identity();
    gamma.block(0, 0, 3, 3) = -identity;

    Covariance<Scalar, 6> process = Covariance<Scalar, 6>::Zero();
    const Scalar sigmaV2 = params.omegaProcessNoise * params.omegaProcessNoise;
    const Scalar sigmaU2 = params.biasProcessNoise * params.biasProcessNoise;
    const Eigen::Matrix<Scalar, 3, 3> crossTerms = (static_cast<Scalar>(0.5) * sigmaU2 * dt * dt) * identity;
    process.block(0, 0, 3, 3) = (sigmaV2 * dt + static_cast<Scalar>(1.0 / 3.0) * sigmaU2 * dt * dt * dt) * identity;
    process.block(0, 3, 3, 3) = crossTerms;
    process.block(3, 0, 3, 3) = crossTerms;
    process.block(3, 3, 3, 3) = (sigmaU2 * dt) * identity;
    
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

        const Eigen::Matrix<Scalar, 3, 3> omegaCross{{0.0, -data.omega(2), data.omega(1)},
                                                        {data.omega(2), 0.0, -data.omega(0)},
                                                        {-data.omega(1), data.omega(0), 0.0}};
        const Eigen::Matrix<Scalar, 3, 3> omegaCross2 = omegaCross * omegaCross;

        phi.block(0, 0, 3, 3) = identity - omegaCross * (static_cast<Scalar>(std::sin(w * dt)) / w)
            + omegaCross2 * ((static_cast<Scalar>(1.0 - std::cos(w * dt))) / w2);
        phi.block(0, 3, 3, 3) = omegaCross * ((static_cast<Scalar>(1.0 - std::cos(w * dt))) / w2)
            - identity * dt - omegaCross2 * ((w * dt - static_cast<Scalar>(std::sin(w * dt))) / w3);
        phi.block(3, 3, 3, 3) = identity;
    }

    data.quat = omega * data.quat;
    data.P = phi * data.P * phi.transpose() + gamma * process * gamma.transpose();
}

/**
* Update the estimated states using 'N' Attitude measurements. Murrell's version of the multiplicative extended Kalman Filter
* (MEKF) update phase is used for improved computational performance. Referenced from "Optimal Estimation of Dynamic Systems"
* by John L. Crassidis and John L. Junkins, 2nd edition.
* Input: data - MEKF data structure
* Output: Boolean if the Kalman filter passed or failed
**/
template <typename Scalar>
inline bool multiplicativeExtendedKalmanUpdate(MEKFData<Scalar>& data)
{
    const OptionalRotationMatrix<Scalar> rotation = quaternionRotationMatrix(data.quat);
    if (!rotation) {
        return false;
    }

    data.deltaX = DeltaStates<Scalar, 6>::Zero();

    // Loop through each of the measurements one at a time for improved computational performance (Murrell's version)
    for (const auto meas : data.attitudeMeasurements) {
        if (!meas.valid) {
            continue;
        }

        const AttitudeVector<Scalar> estMeas = (*rotation) * meas.attitudeRefVector;
        const Eigen::Matrix<Scalar, 3, 3> measCross{{0.0, -estMeas(2), estMeas(1)},
                                                    {estMeas(2), 0.0, -estMeas(0)},
                                                    {-estMeas(1), estMeas(0), 0.0}};

        // Calculate the measurement sensitivity matrix
        Eigen::Matrix<Scalar, 3, 6> H = Eigen::Matrix<Scalar, 3, 6>::Zero();
        H.block(0, 0, 3, 3) = measCross;

        // Calculate the Kalman gain
        Eigen::Matrix<Scalar, 3, 3> S = H * data.P * H.transpose() + meas.sigma * meas.sigma * Eigen::Matrix<Scalar, 3, 3>::Identity();
        Eigen::Matrix<Scalar, 6, 3> K = data.P * H.transpose() * S.inverse();

        // Update states and costates
        const AttitudeVector<Scalar> residual = meas.attitudeMeasVector - estMeas;
        data.deltaX += K * (residual - H * data.deltaX);
        data.P = (Eigen::Matrix<Scalar, 6, 6>::Identity() - K * H) * data.P;
    }

    // Update the primary states
    data.omegaBias += data.deltaX.tail(3);
    data.omega = data.omegaMeas - data.omegaBias;

    const Eigen::Matrix<Scalar, 4, 3> E{{data.quat(3), -data.quat(2), data.quat(1)},
                                        {data.quat(2), data.quat(3), -data.quat(0)},
                                        {-data.quat(1), data.quat(0), data.quat(3)},
                                        {-data.quat(0), -data.quat(1), -data.quat(2)}};
    data.quat += static_cast<Scalar>(0.5) * E * data.deltaX.head(3);
    data.quat /= data.quat.norm();

    return true;
}

/**
* Runs the attitude estimation multiplicative extended Kalman Filter (MEKF) algorithm. Referenced from "Optimal Estimation of Dynamic
* Systems" by John L. Crassidis and John L. Junkins, 2nd edition.
* Input: params - MEKF params
* Input: data - MEKF data structure
* Output: Boolean if the Kalman filter passed or failed
**/
template <typename Scalar>
inline bool multiplicativeExtendedKalmanFilter(const MEKFParams<Scalar>& params, MEKFData<Scalar>& data)
{
    // Propagate the states to 'current time'
    multiplicativeExtendedKalmanPropagate(params, data);

    // Update the states based off the current measurements
    const bool result = multiplicativeExtendedKalmanUpdate(data);

    return result;
}

/**
* Reset the MEKF filter specific data.
* Input: data - The MEKF Filter data structure
* Input: value - Gain for the covariance matrix
* Output:
**/
template <typename Scalar>
inline void multiplicativeExtendedKalmanFilterReset(MEKFData<Scalar>& data, Scalar value)
{
    data.deltaX = DeltaStates<Scalar, 6>::Zero();
    data.P = Covariance<Scalar, 6>::Identity() * value;
    data.initialize = true;
}

} // namespace mekf

} // namespace filter

} // namespace attitude

#endif // ATTITUDE_KALMAN_FILTER_H_
