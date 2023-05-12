#ifndef ATTITUDE_UTILS_H_
#define ATTITUDE_UTILS_H_

#include <cmath>
#include <string>

#include "attitude_libraries/types/typenames.h"

namespace attitude {

/**
 * Converts euler angles into the desired rotation matrix. Referenced from Analytics of Space Systems by Hanspeter Schaub and
 * John L. Junkins, Pages 759 - 760
 * Input: sequence - String which determines the rotation sequence
 * Input: theta - The Euler angles
 * output: Optional rotation matrix
**/
template <typename Scalar>
inline OptionalRotationMatrix<Scalar> eulerRotationMatrix(const std::string& sequence, const EulerAngle<Scalar>& theta)
{
    const Scalar c1 = static_cast<Scalar>(std::cos(theta(0)));
    const Scalar c2 = static_cast<Scalar>(std::cos(theta(1)));
    const Scalar c3 = static_cast<Scalar>(std::cos(theta(2)));
    const Scalar s1 = static_cast<Scalar>(std::sin(theta(0)));
    const Scalar s2 = static_cast<Scalar>(std::sin(theta(1)));
    const Scalar s3 = static_cast<Scalar>(std::sin(theta(2)));

    OptionalRotationMatrix<Scalar> rotation = RotationMatrix<Scalar>::Zero();

    if (sequence.compare("121") == 0) {
        (*rotation)(0, 0) = c2;
        (*rotation)(0, 1) = s2 * s1;
        (*rotation)(0, 2) = -s2 * c1;

        (*rotation)(1, 0) = s3 * s2;
        (*rotation)(1, 1) = -s3 * c2 * s1 + c3 * c1;
        (*rotation)(1, 2) = s3 * c2 * c1 + c3 * s1;

        (*rotation)(2, 0) = c3 * s2;
        (*rotation)(2, 1) = -(c3 * c2 * s1 + s3 * c1);
        (*rotation)(2, 2) = c3 * c2 * c1 - s3 * s1;

        return rotation;
    } else if (sequence.compare("123") == 0) {
        (*rotation)(0, 0) = c3 * c2;
        (*rotation)(0, 1) = c3 * s2 * s1 + s3 * c1;
        (*rotation)(0, 2) = -c3 * s2 * c1 + s3 * s1;

        (*rotation)(1, 0) = -s3 * c2;
        (*rotation)(1, 1) = -s3 * s2 * s1 + c3 * c1;
        (*rotation)(1, 2) = s3 * s2 * c1 + c3 * s1;

        (*rotation)(2, 0) = s2;
        (*rotation)(2, 1) = -c2 * s1;
        (*rotation)(2, 2) = c2 * c1;

        return rotation;
    } else if (sequence.compare("131") == 0) {
        (*rotation)(0, 0) = c2;
        (*rotation)(0, 1) = s2 * c1;
        (*rotation)(0, 2) = s2 * s1;

        (*rotation)(1, 0) = -c3 * s2;
        (*rotation)(1, 1) = c3 * c2 * c1 - s3 * s1;
        (*rotation)(1, 2) = c3 * c2 * s1 + s3 * c1;

        (*rotation)(2, 0) = s3 * s2;
        (*rotation)(2, 1) = -(s3 * c2 * c1 + c3 * s1);
        (*rotation)(2, 2) = -s3 * c2 * s1 + c3 * c1;

        return rotation;
    } else if (sequence.compare("132") == 0) {
        (*rotation)(0, 0) = c3 * c2;
        (*rotation)(0, 1) = c3 * s2 * c1 + s3 * s1;
        (*rotation)(0, 2) = c3 * s2 * s1 - s3 * c1;

        (*rotation)(1, 0) = -s2;
        (*rotation)(1, 1) = c2 * c1;
        (*rotation)(1, 2) = c2 * s1;

        (*rotation)(2, 0) = s3 * c2;
        (*rotation)(2, 1) = s3 * s2 * c1 - c3 * s1;
        (*rotation)(2, 2) = s3 * s2 * s1 + c3 * c1;

        return rotation;
    } else if (sequence.compare("212") == 0) {
        (*rotation)(0, 0) = -s3 * c2 * s1 + c3 * c1;
        (*rotation)(0, 1) = s3 * s2;
        (*rotation)(0, 2) = -(s3 * c2 * c1 + c3 * s1);

        (*rotation)(1, 0) = s2 * s1;
        (*rotation)(1, 1) = c2;
        (*rotation)(1, 2) = s2 * c1;

        (*rotation)(2, 0) = c3 * c2 * s1 + s3 * c1;
        (*rotation)(2, 1) = -c3 * s2;
        (*rotation)(2, 2) = c3 * c2 * c1 - s3 * s1;

        return rotation;
    } else if (sequence.compare("213") == 0) {
        (*rotation)(0, 0) = s3 * s2 * s1 + c3 * c1;
        (*rotation)(0, 1) = s3 * c2;
        (*rotation)(0, 2) = s3 * s2 * c1 - c3 * s1;

        (*rotation)(1, 0) = c3 * s2 * s1 - s3 * c1;
        (*rotation)(1, 1) = c3 * c2;
        (*rotation)(1, 2) = c3 * s2 * c1 + s3 * s1;

        (*rotation)(2, 0) = c2 * s1;
        (*rotation)(2, 1) = -s2;
        (*rotation)(2, 2) = c2 * c1;

        return rotation;
    } else if (sequence.compare("231") == 0) {
        (*rotation)(0, 0) = c2 * c1;
        (*rotation)(0, 1) = s2;
        (*rotation)(0, 2) = -c2 * s1;

        (*rotation)(1, 0) = -c3 * s2 * c1 + s3 * s1;
        (*rotation)(1, 1) = c3 * c2;
        (*rotation)(1, 2) = c3 * s2 * s1 + s3 * c1;

        (*rotation)(2, 0) = s3 * s2 * c1 + c3 * s1;
        (*rotation)(2, 1) = -s3 * c2;
        (*rotation)(2, 2) = -s3 * s2 * s1 + c3 * c1;

        return rotation;
    } else if (sequence.compare("232") == 0) {
        (*rotation)(0, 0) = c3 * c2 * c1 - s3 * s1;
        (*rotation)(0, 1) = c3 * s2;
        (*rotation)(0, 2) = -(c3 * c2 * s1 + s3 * c1);

        (*rotation)(1, 0) = -s2 * c1;
        (*rotation)(1, 1) = c2;
        (*rotation)(1, 2) = s2 * s1;

        (*rotation)(2, 0) = s3 * c2 * c1 + c3 * s1;
        (*rotation)(2, 1) = s3 * s2;
        (*rotation)(2, 2) = -s3 * c2 * s1 + c3 * c1;

        return rotation;
    } else if (sequence.compare("312") == 0) {
        (*rotation)(0, 0) = -s3 * s2 * s1 + c3 * c1;
        (*rotation)(0, 1) = s3 * s2 * c1 + c3 * s1;
        (*rotation)(0, 2) = -s3 * c2;

        (*rotation)(1, 0) = -c2 * s1;
        (*rotation)(1, 1) = c2 * c1;
        (*rotation)(1, 2) = s2;

        (*rotation)(2, 0) = c3 * s2 * s1 + s3 * c1;
        (*rotation)(2, 1) = -c3 * s2 * c1 + s3 * s1;
        (*rotation)(2, 2) = c3 * c2;

        return rotation;
    } else if (sequence.compare("313") == 0) {
        (*rotation)(0, 0) = -s3 * c2 * s1 + c3 * c1;
        (*rotation)(0, 1) = s3 * c2 * c1 + c3 * s1;
        (*rotation)(0, 2) = s3 * s2;

        (*rotation)(1, 0) = -(c3 * c2 * s1 + s3 * c1);
        (*rotation)(1, 1) = c3 * c2 * c1 - s3 * s1;
        (*rotation)(1, 2) = c3 * s2;

        (*rotation)(2, 0) = s2 * s1;
        (*rotation)(2, 1) = -s2 * c1;
        (*rotation)(2, 2) = c2;

        return rotation;
    } else if (sequence.compare("321") == 0) {
        (*rotation)(0, 0) = c2 * c1;
        (*rotation)(0, 1) = c2 * s1;
        (*rotation)(0, 2) = -s2;

        (*rotation)(1, 0) = s3 * s2 * c1 - c3 * s1;
        (*rotation)(1, 1) = s3 * s2 * s1 + c3 * c1;
        (*rotation)(1, 2) = s3 * c2;

        (*rotation)(2, 0) = c3 * s2 * c1 + s3 * s1;
        (*rotation)(2, 1) = c3 * s2 * s1 - s3 * c1;
        (*rotation)(2, 2) = c3 * c2;

        return rotation;
    } else if (sequence.compare("323") == 0) {
        (*rotation)(0, 0) = c3 * c2 * c1 - s3 * s1;
        (*rotation)(0, 1) = c3 * c2 * s1 + s3 * c1;
        (*rotation)(0, 2) = -c3 * s2;

        (*rotation)(1, 0) = -(s3 * c2 * c1 + c3 * s1);
        (*rotation)(1, 1) = -s3 * c2 * s1 + c3 * c1;
        (*rotation)(1, 2) = s3 * s2;

        (*rotation)(2, 0) = s2 * c1;
        (*rotation)(2, 1) = s2 * s1;
        (*rotation)(2, 2) = c2;

        return rotation;
    }
    
    return std::nullopt;
}

/**
 * Converts a rotation matrix into a quaternion. Referenced from S. W. Sheppard, "Quaternion from Rotation Matrix," Journal of
 * Guidance and Control, Vol. 1, No. 3, pp. 223 - 224, 1978
 * Input: rotation - The rotation matrix
 * output: Optional attitude quaternion
**/
template<typename Scalar>
inline OptionalQuaternion<Scalar> rotationToQuaternion(const RotationMatrix<Scalar>& rotation)
{
    Quaternion<Scalar> qTemp = Quaternion<Scalar>::Zero();

    if (rotation(1, 1) > -rotation(2, 2) && rotation(0, 0) > -rotation(1, 1) && rotation(0, 0) > -rotation(2, 2)) {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 + rotation(0, 0) + rotation(1, 1) + rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * value;
        qTemp(1) = static_cast<Scalar>(0.5) * (rotation(1, 2) - rotation(2, 1)) / value;
        qTemp(2) = static_cast<Scalar>(0.5) * (rotation(2, 0) - rotation(0, 2)) / value;
        qTemp(3) = static_cast<Scalar>(0.5) * (rotation(0, 1) - rotation(1, 0)) / value;
    } else if (rotation(1, 1) < -rotation(2, 2) && rotation(0, 0) > rotation(1, 1) && rotation(0, 0) > rotation(2, 2)) {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 + rotation(0, 0) - rotation(1, 1) - rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * (rotation(1, 2) - rotation(2, 1)) / value;
        qTemp(1) = static_cast<Scalar>(0.5) * value;
        qTemp(2) = static_cast<Scalar>(0.5) * (rotation(0, 1) + rotation(1, 0)) / value;
        qTemp(3) = static_cast<Scalar>(0.5) * (rotation(2, 0) + rotation(0, 2)) / value;
    } else if (rotation(1, 1) > rotation(2, 2) && rotation(0, 0) < rotation(1, 1) && rotation(0, 0) < -rotation(2, 2)) {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 - rotation(0, 0) + rotation(1, 1) - rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * (rotation(2, 0) - rotation(0, 2)) / value;
        qTemp(1) = static_cast<Scalar>(0.5) * (rotation(0, 1) + rotation(1, 0)) / value;
        qTemp(2) = static_cast<Scalar>(0.5) * value;
        qTemp(3) = static_cast<Scalar>(0.5) * (rotation(1, 2) + rotation(2, 1)) / value;
    } else {
        const Scalar value = static_cast<Scalar>(std::sqrt(1.0 - rotation(0, 0) - rotation(1, 1) + rotation(2, 2)));

        qTemp(0) = static_cast<Scalar>(0.5) * (rotation(0, 1) - rotation(1, 0)) / value;
        qTemp(1) = static_cast<Scalar>(0.5) * (rotation(2, 0) + rotation(0, 2)) / value;
        qTemp(2) = static_cast<Scalar>(0.5) * (rotation(1, 2) + rotation(2, 1)) / value;
        qTemp(3) = static_cast<Scalar>(0.5) * value;
    }

    OptionalQuaternion<Scalar> quat = Quaternion<Scalar>{qTemp(1), qTemp(2), qTemp(3), qTemp(0)};
    return quat;
}

/**
 * Converts euler angles into a quaternion. Referenced from S. W. Sheppard, "Quaternion from Rotation Matrix," Journal of
 * Guidance and Control, Vol. 1, No. 3, pp. 223 - 224, 1978
 * Input: sequence - String which determines the rotation sequence
 * Input: theta - The Euler angles
 * output: Optional attitude quaternion
**/
template<typename Scalar>
inline OptionalQuaternion<Scalar> eulerToQuaternion(const std::string& sequence, const EulerAngle<Scalar>& theta)
{
    const OptionalRotationMatrix<Scalar> rotation = eulerRotationMatrix(sequence, theta);
    if (!rotation) {
        return std::nullopt;
    }

    return rotationToQuaternion(*rotation);
}

/**
 * Converts the quaternion into a rotation matrix. Referenced from Analytics of Space Systems by Hanspeter Schaub and
 * John L. Junkins and various other sources
 * Input: quat - The attitude quaternion 
 * output: Optional rotation matrix
**/
template<typename Scalar>
inline OptionalRotationMatrix<Scalar> quaternionRotationMatrix(const Quaternion<Scalar>& quat)
{
    // Quaternion vector skew symmetric matrix 
    const Eigen::Matrix<Scalar, 3, 3> qCross{{0.0, -quat(2), quat(1)}, {quat(2), 0.0, -quat(0)}, {-quat(1), quat(0), 0.0}};

    const Eigen::Vector<Scalar, 3> qVector{quat(0), quat(1), quat(2)};                    /// Quaternion vector component
    const Eigen::Matrix<Scalar, 3, 3> identity = Eigen::Matrix<Scalar, 3, 3>::Identity(); /// Identity matrix
    
    OptionalRotationMatrix<Scalar> rotation = RotationMatrix<Scalar>::Zero();
    (*rotation) = (quat(3) * quat(3) - qVector.squaredNorm()) * identity - static_cast<Scalar>(2.0) * quat(3) * qCross
        + static_cast<Scalar>(2.0) * qVector * qVector.transpose();

    return rotation;
}

/**
 * Converts attitude quaternions into the desired euler angle. Referenced from Analytics of Space Systems by Hanspeter
 * Schaub andvJohn L. Junkins, Pages 759 - 760
 * Input: sequence - String which determines the rotation sequence
 * Input: quat - The attitude quaternion
 * output: Optional euler angle in desired sequence
**/
template<typename Scalar>
inline OptionalEulerAngle<Scalar> quaternionToEuler(const std::string& sequence, const Quaternion<Scalar>& quat)
{
    const OptionalRotationMatrix<Scalar> rotation = quaternionRotationMatrix(quat);
    OptionalEulerAngle<Scalar> euler = EulerAngle<Scalar>::Zero();

    if (sequence.compare("121") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(0, 1), -(*rotation)(0, 2)));
        (*euler)(1) = static_cast<Scalar>(std::acos((*rotation)(0, 0)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(1, 0), (*rotation)(2, 0)));

        return euler;
    } else if (sequence.compare("123") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2(-(*rotation)(2, 1), (*rotation)(2, 2)));
        (*euler)(1) = static_cast<Scalar>(std::asin((*rotation)(2, 0)));
        (*euler)(2) = static_cast<Scalar>(std::atan2(-(*rotation)(1, 0), (*rotation)(0, 0)));

        return euler;
    } else if (sequence.compare("131") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(0, 2), (*rotation)(0, 1)));
        (*euler)(1) = static_cast<Scalar>(std::acos((*rotation)(0, 0)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(2, 0), -(*rotation)(1, 0)));

        return euler;
    } else if (sequence.compare("132") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(1, 2), (*rotation)(1, 1)));
        (*euler)(1) = static_cast<Scalar>(std::asin(-(*rotation)(1, 0)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(2, 0), (*rotation)(0, 0)));

        return euler;
    } else if (sequence.compare("212") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(1, 0), (*rotation)(1, 2)));
        (*euler)(1) = static_cast<Scalar>(std::acos((*rotation)(1, 1)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(0, 1), -(*rotation)(2, 1)));

        return euler;
    } else if (sequence.compare("213") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(2, 0), (*rotation)(2, 2)));
        (*euler)(1) = static_cast<Scalar>(std::asin(-(*rotation)(2, 1)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(0, 1), (*rotation)(1, 1)));

        return euler;
    } else if (sequence.compare("231") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2(-(*rotation)(0, 2), (*rotation)(0, 0)));
        (*euler)(1) = static_cast<Scalar>(std::asin((*rotation)(0, 1)));
        (*euler)(2) = static_cast<Scalar>(std::atan2(-(*rotation)(2, 1), (*rotation)(1, 1)));

        return euler;
    } else if (sequence.compare("232") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(1, 2), -(*rotation)(1, 0)));
        (*euler)(1) = static_cast<Scalar>(std::acos((*rotation)(1, 1)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(2, 1), (*rotation)(0, 1)));

        return euler;
    } else if (sequence.compare("312") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2(-(*rotation)(1, 0), (*rotation)(1, 1)));
        (*euler)(1) = static_cast<Scalar>(std::asin((*rotation)(1, 2)));
        (*euler)(2) = static_cast<Scalar>(std::atan2(-(*rotation)(0, 2), (*rotation)(2, 2)));

        return euler;
    } else if (sequence.compare("313") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(2, 0), -(*rotation)(2, 1)));
        (*euler)(1) = static_cast<Scalar>(std::acos((*rotation)(2, 2)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(0, 2), (*rotation)(1, 2)));

        return euler;
    } else if (sequence.compare("321") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(0, 1), (*rotation)(0, 0)));
        (*euler)(1) = static_cast<Scalar>(std::asin(-(*rotation)(0, 2)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(1, 2), (*rotation)(2, 2)));

        return euler;
    } else if (sequence.compare("323") == 0) {
        (*euler)(0) = static_cast<Scalar>(std::atan2((*rotation)(2, 1), (*rotation)(2, 0)));
        (*euler)(1) = static_cast<Scalar>(std::acos((*rotation)(2, 2)));
        (*euler)(2) = static_cast<Scalar>(std::atan2((*rotation)(1, 2), -(*rotation)(0, 2)));

        return euler;
    }

    return std::nullopt;
}

} // namespace attitude

#endif // ATTITUDE_UTILS_H_
