#pragma once

#include "i_physics.hpp"
#include "i_sensor.hpp"

#include "../../shared/result.hpp"
#include "../../shared/text_config.hpp"

#include <memory>
#include <optional>
#include <string_view>

namespace ad::simulation {

[[nodiscard]] auto createSensorFromConfig(std::string_view algorithm,
                                          const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<ISensorModel>>;

[[nodiscard]] auto createPhysicsFromConfig(std::string_view algorithm,
                                           const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IPhysicsModel>>;

} // namespace ad::simulation
