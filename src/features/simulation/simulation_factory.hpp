#pragma once

#include "i_physics.hpp"
#include "sensor/lidar_sensor.hpp"
#include "sensor/odometry_sensor.hpp"

#include "../../shared/result.hpp"
#include "../../shared/text_config.hpp"

#include <memory>
#include <optional>
#include <string_view>

namespace ad::simulation {

[[nodiscard]] auto
createLidarSensorFromConfig(std::string_view algorithm,
                            const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<LidarSensor>>;

[[nodiscard]] auto
createOdometrySensorFromConfig(std::string_view algorithm,
                               const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<OdometrySensor>>;

[[nodiscard]] auto createPhysicsFromConfig(std::string_view algorithm,
                                           const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IPhysicsModel>>;

} // namespace ad::simulation
