#ifndef PASSIVITY_BASED_ADAPTIVE_CONTROL_H_
#define PASSIVITY_BASED_ADAPTIVE_CONTROL_H_

#include "attitude_libraries/quaternions/quaternionMath.h"
#include "attitude_libraries/types/controllerTypes.h"

namespace attitude {

namespace control {
/**
* Determines the parameter estimate rate using a gradient method for the passivity-based adaptive control algorithm
* modified to use the deadzone and projection operators. Referenced from "Passivity Based Adaptive Attitude Control
* of Rigid Spacecraft" by O. Egeland, and J. -M. Godhavn, IEEE Transactions
* on Automatic Control, Vol 39, issue No. 4, April 1994
* Input: params - The passivity-based adaptive control params
* Input: s - The 's' passivity vector
* Input: regressor - The regressor matrix
* Input: theta - The parameter estimates
* Output: Optional parameter estimate rate
**/
template <typename Scalar>
inline std::optional<Eigen::Vector<Scalar, 3>> passivityAdaptiveLaws(const PassivityParams<Scalar>& params,
                                                                     const Eigen::Vector<Scalar, 3>& s,
                                                                     const Eigen::Matrix<Scalar, 3, 3>& regressor,
                                                                     const Theta<Scalar> theta)
{
    // Return null if failed to produce
    std::optional<Scalar> deadzone = deadzoneOperator(params.deadzoneParams, s);
    if (!deadzone) {
        return std::nullopt;
    }

    const Eigen::Vector<Scalar, 3> adapationLaws = -params.gammaInv * regressor.transpose() * s * (*deadzone);
    std::optional<Eigen::Vector<Scalar, 3>> thetaDot = projectionOperator(params.projectionParams, theta, adapationLaws);

    return thetaDot;
}

/**
* Calculate the passivity-based adaptive control augmented with the deadzone and projection operators. Referenced from
* "Passivity Based Adaptive Attitude Control of Rigid Spacecraft" by O. Egeland, and J. -M. Godhavn, IEEE Transactions
* on Automatic Control, Vol 39, issue No. 4, April 1994
* Input: params - The passivity-based adaptive control params
* Input: data - The passivity-based adaptive control data structure
* Output: Result boolean if passed or failed
**/
template <typename Scalar>
inline bool passivityBasedAdaptiveControl(const PassivityParams<Scalar>& params, PassivityControlData<Scalar>& data)
{
    // Calculate the errors
    Quaternion<Scalar> quatError = quaternionError(data.quat, data.quatDesired);
    BodyRate<Scalar> omegaError = data.omega - data.omegaDesired;
    Quaternion<Scalar> quatErrorRate = quaternionKinematics(quatError, omegaError);

    const Eigen::Vector<Scalar, 3> quatErrorVector{quatError(0), quatError(1), quatError(2)};
    const Eigen::Vector<Scalar, 3> quatErrorRateVector{quatErrorRate(0), quatErrorRate(1), quatErrorRate(2)};

    // Calculate passivity terms
    const Eigen::Vector<Scalar, 3> s = omegaError + params.lambda * quatErrorVector;
    const Eigen::Vector<Scalar, 3> omegaR = data.omegaDesired - params.lambda * quatErrorVector;
    const Eigen::Vector<Scalar, 3> omegaRRate = data.omegaDotDesired - params.lambda * quatErrorRateVector;

    // Create the regressor matrix
    const Eigen::Matrix<Scalar, 3, 3> regressor{{omegaRRate(0), -(data.omega)(1) * omegaR(2), (data.omega)(2) * omegaR(1)},
                                                {(data.omega)(0) * omegaR(2), omegaRRate(1), -(data.omega)(2) * omegaR(0)},
                                                {-(data.omega)(0) * omegaR(1), (data.omega)(1) * omegaR(0), omegaRRate(2)}};

    // Update parameter estimates
    std::optional<Eigen::Vector<Scalar, 3>> thetaDot = passivityAdaptiveLaws(params, s, regressor, data.theta);
    if (!thetaDot) {
        return false;
    }

    data.theta += (*thetaDot) * params.dt;

    // Calculate the adaptive control
    data.u = regressor * data.theta - params.k * s;

    return true;
}

} // namespace control

} // namespace attitude

#endif // PASSIVITY_BASED_ADAPTIVE_CONTROL_H_
