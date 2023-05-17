#ifndef CONTROLLER_FUNCTION_REGISTRY_H_
#define CONTROLLER_FUNCTION_REGISTRY_H_

// Standard Includes
#include <functional>
#include <string>
#include <unordered_map>

// Controller Includes
#include "attitude_libraries/controllers/passivityBasedAdaptiveControl.h"

namespace registry {

template <typename Scalar>
using controllerFunc = std::function<bool(attitude::BaseParams<Scalar>&, attitude::ControllerData<Scalar>&)>;

template <typename Scalar>
using controllerResetFunc = std::function<void(attitude::ControllerData<Scalar>&)>;

/**
* Create the function registry for the controllers.
* Input:
* Output:
**/
template <typename Scalar>
inline std::unordered_map<std::string, controllerFunc<Scalar>> controllerRegistry()
{
    std::unordered_map<std::string, controllerFunc<Scalar>> registry;

    registry.insert({"passivityBasedAdaptiveControl",
        [](attitude::BaseParams<Scalar>& params, attitude::ControllerData<Scalar>& data) {
            return attitude::control::passivityBasedAdaptiveControl(static_cast<attitude::control::PassivityParams<Scalar>&>(params),
                                                                    static_cast<attitude::control::PassivityControlData<Scalar>&>(data));
        }});

    return registry;
}

/**
* Create the function registry for the controller reset functions.
* Input:
* Output:
**/
template <typename Scalar>
inline std::unordered_map<std::string, controllerResetFunc<Scalar>> controllerResetRegistry()
{
    std::unordered_map<std::string, controllerResetFunc<Scalar>> registry;

    registry.insert({"passivityBasedAdaptiveControlReset",
        [](attitude::ControllerData<Scalar>& data) {
            return attitude::control::passivityBasedAdaptiveControlReset(static_cast<attitude::control::PassivityControlData<Scalar>&>(data));
        }});

    return registry;
}

} // namespace registry

#endif // CONTROLLER_FUNCTION_REGISTRY_H_
