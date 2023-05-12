#ifndef QUATERNION_MATH_H_
#define QUATERNION_MATH_H_

#include "attitude_libraries/types/typenames.h"

namespace attitude {
/**
 * Performs the attitude quaternion inverse.
 * Input: quat - The attitude quaternion
 * output: The attitude quaternion inverse
**/
template<typename Scalar>
inline Quaternion<Scalar> quatInverse(const Quaternion<Scalar>& quat)
{
    return Quaternion<Scalar>{-quat(0), -quat(1), -quat(2), quat(3)};
}

/**
 * Performs the attitude quaternion multiplication operator.
 * Input: quat1 - The first attitude quaternion
 * Input: quat2 - The second attitude quaternion
 * output: The product of two attitude quaternions being multiplied
**/
template <typename Scalar>
inline Quaternion<Scalar> quatMultiply(const Quaternion<Scalar>& quat1, const Quaternion<Scalar>& quat2)
{
    const Eigen::Vector<Scalar, 3> quat1Vector{quat1(0), quat1(1), quat1(2)};
    const Eigen::Vector<Scalar, 3> quat2Vector{quat2(0), quat2(1), quat2(2)};
    const Eigen::Vector<Scalar, 3> crossProduct = quat1Vector.cross(quat2Vector);
    const Scalar dotProduct = quat1Vector.dot(quat2Vector);

    Quaternion<Scalar> quat = Quaternion<Scalar>::Zero();
    quat(0) = quat1(3) * quat2(0) + quat2(3) * quat1(0) - crossProduct(0);
    quat(1) = quat1(3) * quat2(1) + quat2(3) * quat1(1) - crossProduct(1);
    quat(2) = quat1(3) * quat2(2) + quat2(3) * quat1(2) - crossProduct(2);
    quat(3) = quat1(3) * quat2(3) - dotProduct;

    return quat;
}

/**
 * Calculates the error quaternion of quat 1 with respect to quat 2.
 * Input: quat1 - The first attitude quaternion
 * Input: quat2 - The second attitude quaternion
 * output: The error quaternion
**/
template <typename Scalar>
inline Quaternion<Scalar> quaternionError(const Quaternion<Scalar>& quat1, const Quaternion<Scalar>& quat2)
{
    const Quaternion<Scalar> quat2Inverse = quatInverse(quat2);

    return quatMultiply(quat1, quat2Inverse);
}

/**
 * Calculates the attitude quaternion rate.
 * Input: quat - The attitude quaternion
 * Input: omega - The vehicle/body angular rate
 * output: The attitude quaternion rate
**/
template <typename Scalar>
inline Quaternion<Scalar> quaternionKinematics(const Quaternion<Scalar>& quat, const BodyRate<Scalar>& omega)
{
    // E = [q(4)*eye(3,3) + qCross; -q(1:3)']
    const Eigen::Matrix<Scalar, 4, 3> E{{quat(3), -quat(2), quat(1)},
                                        {quat(2), quat(3), -quat(0)},
                                        {-quat(1), quat(0), quat(3)},
                                        {-quat(0), -quat(1), -quat(2)}};

    return static_cast<Scalar>(0.5) * E * omega;
}

} // namespace attitude

#endif // QUATERNION_MATH_H_
