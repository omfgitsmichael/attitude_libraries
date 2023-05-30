#ifndef FILTER_FUNCTION_REGISTRY_H_
#define FILTER_FUNCTION_REGISTRY_H_

// Standard Includes
#include <functional>
#include <string>
#include <unordered_map>

// Filter Includes
#include "attitude_libraries/filters/AHRSKalmanFilter.h"
#include "attitude_libraries/filters/multiplicativeExtendedKalmanFilter.h"

namespace registry {

template <typename Scalar>
using filterFunc = std::function<bool(attitude::BaseParams<Scalar>&, attitude::FilterData<Scalar>&)>;

template <typename Scalar>
using filterResetFunc = std::function<void(attitude::FilterData<Scalar>&, Scalar)>;

/**
* Create the function registry for the filter functions.
* Input:
* Output:
**/
template <typename Scalar>
inline std::unordered_map<std::string, filterFunc<Scalar>> filterRegistry()
{
    std::unordered_map<std::string, filterFunc<Scalar>> registry;

    registry.insert({"AHRSKalmanFilter",
        [](attitude::BaseParams<Scalar>& params, attitude::FilterData<Scalar>& data) {
            return attitude::filter::ahrs::AHRSKalmanFilter(static_cast<attitude::filter::ahrs::AHRSParams<Scalar>&>(params),
                                                            static_cast<attitude::filter::ahrs::AHRSData<Scalar>&>(data));
        }});
    
    registry.insert({"multiplicativeExtendedKalmanFilter",
        [](attitude::BaseParams<Scalar>& params, attitude::FilterData<Scalar>& data) {
            return attitude::filter::mekf::multiplicativeExtendedKalmanFilter(static_cast<attitude::filter::mekf::MEKFParams<Scalar>&>(params),
                                                                              static_cast<attitude::filter::mekf::MEKFData<Scalar>&>(data));
        }});

    return registry;
}

/**
* Create the function registry for the filter reset functions.
* Input:
* Output:
**/
template <typename Scalar>
inline std::unordered_map<std::string, filterResetFunc<Scalar>> filterResetRegistry()
{
    std::unordered_map<std::string, filterResetFunc<Scalar>> registry;

    registry.insert({"AHRSKalmanFilterReset",
        [](attitude::FilterData<Scalar>& data, Scalar value) {
            return attitude::filter::ahrs::AHRSKalmanFilterReset(static_cast<attitude::filter::ahrs::AHRSData<Scalar>&>(data),
                                                                 value);
        }});
    
    registry.insert({"multiplicativeExtendedKalmanFilterReset",
        [](attitude::FilterData<Scalar>& data, Scalar value) {
            return attitude::filter::mekf::multiplicativeExtendedKalmanFilterReset(static_cast<attitude::filter::mekf::MEKFData<Scalar>&>(data),
                                                                                   value);
        }});

    return registry;
}

} // namespace registry

#endif // FILTER_FUNCTION_REGISTRY_H_
