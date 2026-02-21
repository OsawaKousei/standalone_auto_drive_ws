#include "simulation_factory.hpp"

#include "lidar_sim.hpp"
#include "unicycle_model.hpp"

namespace ad::simulation {

namespace {

[[nodiscard]] auto parseLidarConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<LidarSimConfig> {
  auto rayCountValue = config::kDefaultRayCount;
  auto minAngleValue = config::kDefaultMinAngle;
  auto maxAngleValue = config::kDefaultMaxAngle;
  auto maxRangeValue = config::kDefaultMaxRange;
  auto rangeStepValue = config::kDefaultRangeStep;
  if (!configDoc.has_value()) {
    return config::lidarDefaultConfig();
  }

  const auto &cfg = *configDoc;
  const auto rayCount = cfg.findRaw("", "ray_count");
  if (rayCount) {
    const auto parsed = ::ad::config::parseIntValue(*rayCount);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    rayCountValue = *parsed;
  }

  const auto minAngle = cfg.findRaw("", "min_angle");
  if (minAngle) {
    const auto parsed = ::ad::config::parseDoubleValue(*minAngle);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    minAngleValue = *parsed;
  }

  const auto maxAngle = cfg.findRaw("", "max_angle");
  if (maxAngle) {
    const auto parsed = ::ad::config::parseDoubleValue(*maxAngle);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxAngleValue = *parsed;
  }

  const auto maxRange = cfg.findRaw("", "max_range");
  if (maxRange) {
    const auto parsed = ::ad::config::parseDoubleValue(*maxRange);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxRangeValue = *parsed;
  }

  const auto rangeStep = cfg.findRaw("", "range_step");
  if (rangeStep) {
    const auto parsed = ::ad::config::parseDoubleValue(*rangeStep);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    rangeStepValue = *parsed;
  }

  return LidarSimConfig{.rayCount = rayCountValue,
                        .minAngle = minAngleValue,
                        .maxAngle = maxAngleValue,
                        .maxRange = maxRangeValue,
                        .rangeStep = rangeStepValue};
}

[[nodiscard]] auto parseUnicycleConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<UnicycleModelConfig> {
  auto maxLinearSpeedValue = config::kDefaultMaxLinearSpeed;
  auto maxAngularSpeedValue = config::kDefaultMaxAngularSpeed;
  if (!configDoc.has_value()) {
    return config::unicycleDefaultConfig();
  }

  const auto &cfg = *configDoc;
  const auto linear = cfg.findRaw("", "max_linear_speed");
  if (linear) {
    const auto parsed = ::ad::config::parseDoubleValue(*linear);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxLinearSpeedValue = *parsed;
  }

  const auto angular = cfg.findRaw("", "max_angular_speed");
  if (angular) {
    const auto parsed = ::ad::config::parseDoubleValue(*angular);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxAngularSpeedValue = *parsed;
  }

  return UnicycleModelConfig{.maxLinearSpeed = maxLinearSpeedValue,
                             .maxAngularSpeed = maxAngularSpeedValue};
}

} // namespace

auto createSensorFromConfig(std::string_view algorithm,
                            const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<ISensorModel>> {
  if (algorithm != "lidar") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported sensor algorithm: " + std::string{algorithm}});
  }

  const auto configValue = parseLidarConfig(configDoc);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::unique_ptr<ISensorModel>{new LidarSim{*configValue}};
}

auto createPhysicsFromConfig(std::string_view algorithm,
                             const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IPhysicsModel>> {
  if (algorithm != "unicycle") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported physics algorithm: " + std::string{algorithm}});
  }

  const auto configValue = parseUnicycleConfig(configDoc);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::unique_ptr<IPhysicsModel>{new UnicycleModel{*configValue}};
}

} // namespace ad::simulation
