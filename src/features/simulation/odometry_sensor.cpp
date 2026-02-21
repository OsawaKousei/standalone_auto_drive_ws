#include "odometry_sensor.hpp"

#include <cmath>
#include <numbers>

namespace {

constexpr auto kAnglePeriod = 2.0 * std::numbers::pi;

[[nodiscard]] auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, kAnglePeriod);
  if (angle < 0.0) {
    angle += kAnglePeriod;
  }
  return angle - std::numbers::pi;
}

} // namespace

namespace ad::simulation {

OdometrySensor::OdometrySensor(OdometrySensorConfig config)
    : config_(config), generator_(static_cast<std::mt19937::result_type>(config.seed)) {}

auto OdometrySensor::measure(const types::Pose &previousPose, const types::Pose &currentPose) const
    -> Result<types::OdometryDelta> {
  if (!std::isfinite(previousPose.x) || !std::isfinite(previousPose.y) ||
      !std::isfinite(previousPose.theta) || !std::isfinite(currentPose.x) ||
      !std::isfinite(currentPose.y) || !std::isfinite(currentPose.theta)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Pose values must be finite."});
  }

  if (config_.forwardNoiseStddev < 0.0 || config_.lateralNoiseStddev < 0.0 ||
      config_.thetaNoiseStddev < 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Odometry noise stddev must be non-negative."});
  }

  const auto deltaXWorld = currentPose.x - previousPose.x;
  const auto deltaYWorld = currentPose.y - previousPose.y;
  const auto cosTheta = std::cos(previousPose.theta);
  const auto sinTheta = std::sin(previousPose.theta);

  auto deltaForward = (cosTheta * deltaXWorld) + (sinTheta * deltaYWorld);
  auto deltaLateral = (-sinTheta * deltaXWorld) + (cosTheta * deltaYWorld);
  auto deltaTheta = normalizeAngle(currentPose.theta - previousPose.theta);

  const auto sampleNoise = [&](double stddev) {
    if (stddev == 0.0) {
      return 0.0;
    }
    auto distribution = std::normal_distribution<double>{0.0, stddev};
    return distribution(generator_);
  };

  deltaForward += sampleNoise(config_.forwardNoiseStddev);
  deltaLateral += sampleNoise(config_.lateralNoiseStddev);
  deltaTheta = normalizeAngle(deltaTheta + sampleNoise(config_.thetaNoiseStddev));

  return types::OdometryDelta{
      .deltaForward = deltaForward, .deltaLateral = deltaLateral, .deltaTheta = deltaTheta};
}

} // namespace ad::simulation
