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

} // namespace registry

#endif // FILTER_FUNCTION_REGISTRY_H_
