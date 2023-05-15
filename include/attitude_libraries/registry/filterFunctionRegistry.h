#ifndef FILTER_FUNCTION_REGISTRY_H_
#define FILTER_FUNCTION_REGISTRY_H_

// Standard Includes
#include <functional>
#include <string>
#include <unordered_map>

// Filter Includes
#include "filters/AHRSKalmanFilter.h"
#include "filters/multiplicativeExtendedKalmanFilter.h"

namespace registry {

template <typename Scalar>
using func = std::function<bool(attitude::BaseParams<Scalar>&, attitude::BaseData<Scalar>&)>;

template <typename Scalar>
inline std::unordered_map<std::string, func<Scalar>> filterRegistry()
{
    std::unordered_map<std::string, func<Scalar>> registry;

    registry.insert({"AHRSKalmanFilter",
        [](attitude::BaseParams<Scalar>& params, attitude::BaseData<Scalar>& data) {
            return attitude::filter::ahrs::AHRSKalmanFilter(static_cast<attitude::filter::ahrs::AHRSParams<Scalar>&>(params),
                                                            static_cast<attitude::filter::ahrs::AHRSData<Scalar>&>(data));
        }});
    
    registry.insert({"multiplicativeExtendedKalmanFilter",
        [](attitude::BaseParams<Scalar>& params, attitude::BaseData<Scalar>& data) {
            return attitude::filter::mekf::multiplicativeExtendedKalmanFilter(static_cast<attitude::filter::mekf::MEKFParams<Scalar>&>(params),
                                                                              static_cast<attitude::filter::mekf::MEKFData<Scalar>&>(data));
        }});

    return registry;
}

} // namespace registry

#endif // FILTER_FUNCTION_REGISTRY_H_
