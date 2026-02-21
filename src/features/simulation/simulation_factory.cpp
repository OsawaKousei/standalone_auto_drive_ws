#include "simulation_factory.hpp"

#include "lidar_sim.hpp"
#include "odometry_sensor.hpp"
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

[[nodiscard]] auto parseOdometryConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<OdometrySensorConfig> {
  auto forwardNoiseStddevValue = config::kDefaultForwardNoiseStddev;
  auto lateralNoiseStddevValue = config::kDefaultLateralNoiseStddev;
  auto thetaNoiseStddevValue = config::kDefaultThetaNoiseStddev;
  auto seedValue = config::kDefaultSeed;
  if (!configDoc.has_value()) {
    return config::odometryDefaultConfig();
  }

  const auto &cfg = *configDoc;
  const auto forwardNoiseStddev = cfg.findRaw("", "forward_noise_stddev");
  if (forwardNoiseStddev) {
    const auto parsed = ::ad::config::parseDoubleValue(*forwardNoiseStddev);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    forwardNoiseStddevValue = *parsed;
  }

  const auto lateralNoiseStddev = cfg.findRaw("", "lateral_noise_stddev");
  if (lateralNoiseStddev) {
    const auto parsed = ::ad::config::parseDoubleValue(*lateralNoiseStddev);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    lateralNoiseStddevValue = *parsed;
  }

  const auto thetaNoiseStddev = cfg.findRaw("", "theta_noise_stddev");
  if (thetaNoiseStddev) {
    const auto parsed = ::ad::config::parseDoubleValue(*thetaNoiseStddev);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    thetaNoiseStddevValue = *parsed;
  }

  const auto seed = cfg.findRaw("", "seed");
  if (seed) {
    const auto parsed = ::ad::config::parseIntValue(*seed);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    seedValue = *parsed;
  }

  return OdometrySensorConfig{.forwardNoiseStddev = forwardNoiseStddevValue,
                              .lateralNoiseStddev = lateralNoiseStddevValue,
                              .thetaNoiseStddev = thetaNoiseStddevValue,
                              .seed = seedValue};
}

} // namespace

auto createLidarSensorFromConfig(std::string_view algorithm,
                                 const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<ILidarSensor>> {
  if (algorithm != "lidar") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported lidar sensor algorithm: " + std::string{algorithm}});
  }

  const auto configValue = parseLidarConfig(configDoc);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::make_unique<LidarSim>(*configValue);
}

auto createOdometrySensorFromConfig(std::string_view algorithm,
                                    const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IOdometrySensor>> {
  if (algorithm != "odometry") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported odometry sensor algorithm: " + std::string{algorithm}});
  }

  const auto configValue = parseOdometryConfig(configDoc);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::make_unique<OdometrySensor>(*configValue);
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

  return std::make_unique<UnicycleModel>(*configValue);
}

} // namespace ad::simulation
