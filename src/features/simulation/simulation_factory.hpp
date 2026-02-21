#pragma once

#include "i_physics.hpp"
#include "i_sensor.hpp"
#include "odometry_sensor.hpp"

#include "../../shared/result.hpp"
#include "../../shared/text_config.hpp"

#include <memory>
#include <optional>
#include <string_view>

namespace ad::simulation {

[[nodiscard]] auto
createLidarSensorFromConfig(std::string_view algorithm,
                            const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<ILidarSensor>>;

[[nodiscard]] auto
createOdometrySensorFromConfig(std::string_view algorithm,
                               const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IOdometrySensor>>;

[[nodiscard]] auto createPhysicsFromConfig(std::string_view algorithm,
                                           const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IPhysicsModel>>;

} // namespace ad::simulation
