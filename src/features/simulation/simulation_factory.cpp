#include "simulation_factory.hpp"

#include "lidar_sensor.hpp"
#include "odometry_sensor.hpp"
#include "unicycle_model.hpp"

namespace ad::simulation {

namespace {

[[nodiscard]] auto requiredRaw(const ::ad::config::TextConfig &cfg, std::string_view key,
                               std::string_view domain) -> Result<std::string_view> {
  const auto raw = cfg.findRaw("", key);
  if (!raw) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Required " + std::string{domain} +
                                                " config key is missing: " + std::string{key}});
  }
  return *raw;
}

[[nodiscard]] auto parseLidarConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<LidarSensorConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Lidar config is required."});
  }

  const auto &cfg = *configDoc;
  const auto rayCount = requiredRaw(cfg, "ray_count", "lidar");
  if (!rayCount) {
    return tl::make_unexpected(rayCount.error());
  }
  const auto rayCountValue = ::ad::config::parseIntValue(*rayCount);
  if (!rayCountValue) {
    return tl::make_unexpected(rayCountValue.error());
  }

  const auto minAngle = requiredRaw(cfg, "min_angle", "lidar");
  if (!minAngle) {
    return tl::make_unexpected(minAngle.error());
  }
  const auto minAngleValue = ::ad::config::parseDoubleValue(*minAngle);
  if (!minAngleValue) {
    return tl::make_unexpected(minAngleValue.error());
  }

  const auto maxAngle = requiredRaw(cfg, "max_angle", "lidar");
  if (!maxAngle) {
    return tl::make_unexpected(maxAngle.error());
  }
  const auto maxAngleValue = ::ad::config::parseDoubleValue(*maxAngle);
  if (!maxAngleValue) {
    return tl::make_unexpected(maxAngleValue.error());
  }

  const auto maxRange = requiredRaw(cfg, "max_range", "lidar");
  if (!maxRange) {
    return tl::make_unexpected(maxRange.error());
  }
  const auto maxRangeValue = ::ad::config::parseDoubleValue(*maxRange);
  if (!maxRangeValue) {
    return tl::make_unexpected(maxRangeValue.error());
  }

  const auto rangeStep = requiredRaw(cfg, "range_step", "lidar");
  if (!rangeStep) {
    return tl::make_unexpected(rangeStep.error());
  }
  const auto rangeStepValue = ::ad::config::parseDoubleValue(*rangeStep);
  if (!rangeStepValue) {
    return tl::make_unexpected(rangeStepValue.error());
  }

  return LidarSensorConfig{.rayCount = *rayCountValue,
                           .minAngle = *minAngleValue,
                           .maxAngle = *maxAngleValue,
                           .maxRange = *maxRangeValue,
                           .rangeStep = *rangeStepValue};
}

[[nodiscard]] auto parseUnicycleConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<UnicycleModelConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Physics config is required."});
  }

  const auto &cfg = *configDoc;
  const auto linear = requiredRaw(cfg, "max_linear_speed", "physics");
  if (!linear) {
    return tl::make_unexpected(linear.error());
  }
  const auto maxLinearSpeedValue = ::ad::config::parseDoubleValue(*linear);
  if (!maxLinearSpeedValue) {
    return tl::make_unexpected(maxLinearSpeedValue.error());
  }

  const auto angular = requiredRaw(cfg, "max_angular_speed", "physics");
  if (!angular) {
    return tl::make_unexpected(angular.error());
  }
  const auto maxAngularSpeedValue = ::ad::config::parseDoubleValue(*angular);
  if (!maxAngularSpeedValue) {
    return tl::make_unexpected(maxAngularSpeedValue.error());
  }

  return UnicycleModelConfig{.maxLinearSpeed = *maxLinearSpeedValue,
                             .maxAngularSpeed = *maxAngularSpeedValue};
}

[[nodiscard]] auto parseOdometryConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<OdometrySensorConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Odometry config is required."});
  }

  const auto &cfg = *configDoc;
  const auto forwardNoiseStddev = requiredRaw(cfg, "forward_noise_stddev", "odometry");
  if (!forwardNoiseStddev) {
    return tl::make_unexpected(forwardNoiseStddev.error());
  }
  const auto forwardNoiseStddevValue = ::ad::config::parseDoubleValue(*forwardNoiseStddev);
  if (!forwardNoiseStddevValue) {
    return tl::make_unexpected(forwardNoiseStddevValue.error());
  }

  const auto lateralNoiseStddev = requiredRaw(cfg, "lateral_noise_stddev", "odometry");
  if (!lateralNoiseStddev) {
    return tl::make_unexpected(lateralNoiseStddev.error());
  }
  const auto lateralNoiseStddevValue = ::ad::config::parseDoubleValue(*lateralNoiseStddev);
  if (!lateralNoiseStddevValue) {
    return tl::make_unexpected(lateralNoiseStddevValue.error());
  }

  const auto thetaNoiseStddev = requiredRaw(cfg, "theta_noise_stddev", "odometry");
  if (!thetaNoiseStddev) {
    return tl::make_unexpected(thetaNoiseStddev.error());
  }
  const auto thetaNoiseStddevValue = ::ad::config::parseDoubleValue(*thetaNoiseStddev);
  if (!thetaNoiseStddevValue) {
    return tl::make_unexpected(thetaNoiseStddevValue.error());
  }

  const auto seed = requiredRaw(cfg, "seed", "odometry");
  if (!seed) {
    return tl::make_unexpected(seed.error());
  }
  const auto seedValue = ::ad::config::parseIntValue(*seed);
  if (!seedValue) {
    return tl::make_unexpected(seedValue.error());
  }

  return OdometrySensorConfig{.forwardNoiseStddev = *forwardNoiseStddevValue,
                              .lateralNoiseStddev = *lateralNoiseStddevValue,
                              .thetaNoiseStddev = *thetaNoiseStddevValue,
                              .seed = *seedValue};
}

} // namespace

auto createLidarSensorFromConfig(std::string_view algorithm,
                                 const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<LidarSensor>> {
  if (algorithm != "lidar") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported lidar sensor algorithm: " + std::string{algorithm}});
  }

  const auto configValue = parseLidarConfig(configDoc);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::make_unique<LidarSensor>(*configValue);
}

auto createOdometrySensorFromConfig(std::string_view algorithm,
                                    const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<OdometrySensor>> {
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
