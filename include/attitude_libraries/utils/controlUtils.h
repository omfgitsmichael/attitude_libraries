#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

#include <algorithm>
#include <optional>

#include <eigen3/Eigen/Dense>

namespace attitude {

/**
 * Deadzone operator parameters
**/
template <typename Scalar>
struct DeadzoneParams
{
    Scalar del = 0.0;
    Scalar e0 = 0.0;
};

/**
 * Projection operator parameters
**/
template <typename Scalar, int Size>
struct ProjectionParams
{
    Eigen::Vector<Scalar, Size> epsilon = Eigen::Vector<Scalar, Size>::Zero();
    Eigen::Vector<Scalar, Size> thetaMax = Eigen::Vector<Scalar, Size>::Zero();
};

/**
 * The continuous deadzone operator that ensures bounded parameters when subject to measurement noise. Stops the adaptation laws
 * when the norm of the tracking error becomes smaller than the prescribed e0. 0 < params.del < 1
 * Referenced from "Robust and Adaptive Control with Aerospace Applications" by Eugene Lavretsky and Kevin A. Wise, chapter 11
 * Input: params - Deadzone params
 * Input: error - Tracking error vector
 * output: Optional deadzone operator
**/
template <typename Scalar, int Size>
inline std::optional<Scalar> deadzoneOperator(const DeadzoneParams<Scalar>& params, const Eigen::Vector<Scalar, Size>& error)
{
    // If the parameters aren't within range return null
    if (params.del <= 0.0 || params.del >= 1.0 || params.e0 <= 0.0) {
        return std::nullopt;
    }

    Scalar e = error.norm();

    std::optional<Scalar> deadzone = std::make_optional<Scalar>();
    *deadzone = (e - params.del * params.e0) / ((1.0 - params.del) * params.e0);
    *deadzone = std::max(0.0, std::min(1.0, *deadzone));

    return deadzone;
}

/**
 * The generic projection operator that works for NxM parameter estimate matrices and not just vectors. The projection operator 
 * acts as an anti-windup method for the parameter estimates and bounds them. epsilon > 0, thetaMax > 0
 * Referenced from "Robust and Adaptive Control with Aerospace Applications" by Eugene Lavretsky and Kevin A. Wise, chapter 11
 * Input: params - projection params
 * Input: theta - Parameter estimates
 * Input: adaptationLaw - Parameter estimate adaptation laws
 * output: Optional theta parameter estimate rate
**/
template <typename Scalar, int Size1, int Size2>
inline std::optional<Eigen::Matrix<Scalar, Size1, Size2>> projectionOperator(const ProjectionParams<Scalar, Size2>& params,
                                                                             const Eigen::Matrix<Scalar, Size1, Size2>& theta,
                                                                             const Eigen::Matrix<Scalar, Size1, Size2>& adaptationLaw)
{
    unsigned int cols = theta.cols();

    // If any of the parameters fail return null
    for (unsigned int i = 0; i < cols; i++) {
        if (params.epsilon(i) <= 0.0 || params.thetaMax(i) <= 0.0) {
            return std::nullopt;
        }
    }

    std::optional<Eigen::Matrix<Scalar, Size1, Size2>> thetaDot = Eigen::Matrix<Scalar, Size1, Size2>::Zero();

    for (unsigned int i = 0; i < cols; i++) {
        const Scalar F = ((1.0 + params.epsilon(i)) * theta.col(i).squaredNorm() - params.thetaMax(i) * params.thetaMax(i)) 
            / (params.epsilon(i) * params.thetaMax(i) * params.thetaMax(i));
    
        const Eigen::Vector<Scalar, Size1> deltaF = 2.0 * ((1.0 + params.epsilon(i)) / (params.epsilon(i) * params.thetaMax(i))) * theta.col(i);

        if (F > 0.0 && adaptationLaw.col(i).transpose() * deltaF > 0.0) {
            (*thetaDot).col(i) = adaptationLaw.col(i) - (deltaF * deltaF.transpose() / deltaF.squaredNorm()) * adaptationLaw.col(i) * F;
        } else {
            (*thetaDot).col(i) = adaptationLaw.col(i);
        }
    }

    return thetaDot;
}

} // namespace attitude

#endif // CONTROL_UTILS_H_
