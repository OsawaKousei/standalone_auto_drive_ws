#include "simulation_factory.hpp"

#include "physics/holonomic_model.hpp"
#include "physics/unicycle_model.hpp"
#include "sensor/lidar_sensor.hpp"
#include "sensor/odometry_sensor.hpp"

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

  const auto rangeNoiseStddev = requiredRaw(cfg, "range_noise_stddev", "lidar");
  if (!rangeNoiseStddev) {
    return tl::make_unexpected(rangeNoiseStddev.error());
  }
  const auto rangeNoiseStddevValue = ::ad::config::parseDoubleValue(*rangeNoiseStddev);
  if (!rangeNoiseStddevValue) {
    return tl::make_unexpected(rangeNoiseStddevValue.error());
  }

  return LidarSensorConfig{.rayCount = *rayCountValue,
                           .minAngle = *minAngleValue,
                           .maxAngle = *maxAngleValue,
                           .maxRange = *maxRangeValue,
                           .rangeStep = *rangeStepValue,
                           .rangeNoiseStddev = *rangeNoiseStddevValue};
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

  const auto linearAcceleration = requiredRaw(cfg, "max_linear_acceleration", "physics");
  if (!linearAcceleration) {
    return tl::make_unexpected(linearAcceleration.error());
  }
  const auto maxLinearAccelerationValue = ::ad::config::parseDoubleValue(*linearAcceleration);
  if (!maxLinearAccelerationValue) {
    return tl::make_unexpected(maxLinearAccelerationValue.error());
  }

  const auto angularAcceleration = requiredRaw(cfg, "max_angular_acceleration", "physics");
  if (!angularAcceleration) {
    return tl::make_unexpected(angularAcceleration.error());
  }
  const auto maxAngularAccelerationValue = ::ad::config::parseDoubleValue(*angularAcceleration);
  if (!maxAngularAccelerationValue) {
    return tl::make_unexpected(maxAngularAccelerationValue.error());
  }

  const auto tauLinear = requiredRaw(cfg, "tau_linear", "physics");
  if (!tauLinear) {
    return tl::make_unexpected(tauLinear.error());
  }
  const auto tauLinearValue = ::ad::config::parseDoubleValue(*tauLinear);
  if (!tauLinearValue) {
    return tl::make_unexpected(tauLinearValue.error());
  }

  const auto tauAngular = requiredRaw(cfg, "tau_angular", "physics");
  if (!tauAngular) {
    return tl::make_unexpected(tauAngular.error());
  }
  const auto tauAngularValue = ::ad::config::parseDoubleValue(*tauAngular);
  if (!tauAngularValue) {
    return tl::make_unexpected(tauAngularValue.error());
  }

  return UnicycleModelConfig{.maxLinearSpeed = *maxLinearSpeedValue,
                             .maxAngularSpeed = *maxAngularSpeedValue,
                             .maxLinearAcceleration = *maxLinearAccelerationValue,
                             .maxAngularAcceleration = *maxAngularAccelerationValue,
                             .tauLinear = *tauLinearValue,
                             .tauAngular = *tauAngularValue};
}

[[nodiscard]] auto parseHolonomicConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<HolonomicModelConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Physics config is required."});
  }

  const auto &cfg = *configDoc;

  const auto maxLinearSpeedX = requiredRaw(cfg, "max_linear_speed_x", "physics");
  if (!maxLinearSpeedX) {
    return tl::make_unexpected(maxLinearSpeedX.error());
  }
  const auto maxLinearSpeedXValue = ::ad::config::parseDoubleValue(*maxLinearSpeedX);
  if (!maxLinearSpeedXValue) {
    return tl::make_unexpected(maxLinearSpeedXValue.error());
  }

  const auto maxLinearSpeedY = requiredRaw(cfg, "max_linear_speed_y", "physics");
  if (!maxLinearSpeedY) {
    return tl::make_unexpected(maxLinearSpeedY.error());
  }
  const auto maxLinearSpeedYValue = ::ad::config::parseDoubleValue(*maxLinearSpeedY);
  if (!maxLinearSpeedYValue) {
    return tl::make_unexpected(maxLinearSpeedYValue.error());
  }

  const auto maxAngularSpeed = requiredRaw(cfg, "max_angular_speed", "physics");
  if (!maxAngularSpeed) {
    return tl::make_unexpected(maxAngularSpeed.error());
  }
  const auto maxAngularSpeedValue = ::ad::config::parseDoubleValue(*maxAngularSpeed);
  if (!maxAngularSpeedValue) {
    return tl::make_unexpected(maxAngularSpeedValue.error());
  }

  const auto maxLinearAccelerationX = requiredRaw(cfg, "max_linear_acceleration_x", "physics");
  if (!maxLinearAccelerationX) {
    return tl::make_unexpected(maxLinearAccelerationX.error());
  }
  const auto maxLinearAccelerationXValue = ::ad::config::parseDoubleValue(*maxLinearAccelerationX);
  if (!maxLinearAccelerationXValue) {
    return tl::make_unexpected(maxLinearAccelerationXValue.error());
  }

  const auto maxLinearAccelerationY = requiredRaw(cfg, "max_linear_acceleration_y", "physics");
  if (!maxLinearAccelerationY) {
    return tl::make_unexpected(maxLinearAccelerationY.error());
  }
  const auto maxLinearAccelerationYValue = ::ad::config::parseDoubleValue(*maxLinearAccelerationY);
  if (!maxLinearAccelerationYValue) {
    return tl::make_unexpected(maxLinearAccelerationYValue.error());
  }

  const auto maxAngularAcceleration = requiredRaw(cfg, "max_angular_acceleration", "physics");
  if (!maxAngularAcceleration) {
    return tl::make_unexpected(maxAngularAcceleration.error());
  }
  const auto maxAngularAccelerationValue = ::ad::config::parseDoubleValue(*maxAngularAcceleration);
  if (!maxAngularAccelerationValue) {
    return tl::make_unexpected(maxAngularAccelerationValue.error());
  }

  const auto tauLinearX = requiredRaw(cfg, "tau_linear_x", "physics");
  if (!tauLinearX) {
    return tl::make_unexpected(tauLinearX.error());
  }
  const auto tauLinearXValue = ::ad::config::parseDoubleValue(*tauLinearX);
  if (!tauLinearXValue) {
    return tl::make_unexpected(tauLinearXValue.error());
  }

  const auto tauLinearY = requiredRaw(cfg, "tau_linear_y", "physics");
  if (!tauLinearY) {
    return tl::make_unexpected(tauLinearY.error());
  }
  const auto tauLinearYValue = ::ad::config::parseDoubleValue(*tauLinearY);
  if (!tauLinearYValue) {
    return tl::make_unexpected(tauLinearYValue.error());
  }

  const auto tauAngular = requiredRaw(cfg, "tau_angular", "physics");
  if (!tauAngular) {
    return tl::make_unexpected(tauAngular.error());
  }
  const auto tauAngularValue = ::ad::config::parseDoubleValue(*tauAngular);
  if (!tauAngularValue) {
    return tl::make_unexpected(tauAngularValue.error());
  }

  return HolonomicModelConfig{.maxLinearSpeedX = *maxLinearSpeedXValue,
                              .maxLinearSpeedY = *maxLinearSpeedYValue,
                              .maxAngularSpeed = *maxAngularSpeedValue,
                              .maxLinearAccelerationX = *maxLinearAccelerationXValue,
                              .maxLinearAccelerationY = *maxLinearAccelerationYValue,
                              .maxAngularAcceleration = *maxAngularAccelerationValue,
                              .tauLinearX = *tauLinearXValue,
                              .tauLinearY = *tauLinearYValue,
                              .tauAngular = *tauAngularValue};
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

  return OdometrySensorConfig{.forwardNoiseStddev = *forwardNoiseStddevValue,
                              .lateralNoiseStddev = *lateralNoiseStddevValue,
                              .thetaNoiseStddev = *thetaNoiseStddevValue};
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
  if (algorithm == "unicycle") {
    const auto configValue = parseUnicycleConfig(configDoc);
    if (!configValue) {
      return tl::make_unexpected(configValue.error());
    }

    return std::make_unique<UnicycleModel>(*configValue);
  }

  if (algorithm == "holonomic") {
    const auto configValue = parseHolonomicConfig(configDoc);
    if (!configValue) {
      return tl::make_unexpected(configValue.error());
    }

    return std::make_unique<HolonomicModel>(*configValue);
  }

  return tl::make_unexpected(
      Error{.code = ErrorCode::InvalidInput,
            .message = "Unsupported physics algorithm: " + std::string{algorithm}});
}

} // namespace ad::simulation
