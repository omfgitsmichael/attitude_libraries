#ifndef CONTROLLER_TYPES_H_
#define CONTROLLER_TYPES_H_

#include "attitude_libraries/types/typenames.h"
#include "attitude_libraries/utils/controlUtils.h"

namespace attitude {

namespace control {
/**
 * System parameter estimate typename
**/
template <typename Scalar>
using Theta = Eigen::Vector<Scalar, 3>;

/**
* The passivity-based adaptive control paramters
**/
template <typename Scalar>
struct PassivityParams : BaseParams<Scalar>
{
    Eigen::Matrix<Scalar, 3, 3> lambda = Eigen::Matrix<Scalar, 3, 3>::Zero();   /// Diaginal matrix
    Eigen::Matrix<Scalar, 3, 3> k = Eigen::Matrix<Scalar, 3, 3>::Zero();        /// Diaginal matrix
    Eigen::Matrix<Scalar, 3, 3> gammaInv = Eigen::Matrix<Scalar, 3, 3>::Zero(); /// Diaginal matrix
    DeadzoneParams<Scalar> deadzoneParams;                                      /// Deadzone parameters
    ProjectionParams<Scalar, 1> projectionParams;                               /// Projection parameters
};

/**
 * Passivity-based adpative control data type structure
**/
template <typename Scalar>
struct PassivityControlData : ControllerData<Scalar>
{
    control::Theta<Scalar> theta = control::Theta<Scalar>::Zero();
};

} // namespace control

} // namespace attitude

#endif // CONTROLLER_TYPES_H_
