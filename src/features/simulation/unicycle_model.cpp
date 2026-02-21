#include "unicycle_model.hpp"

#include <algorithm>
#include <cmath>

namespace ad::simulation {

UnicycleModel::UnicycleModel(UnicycleModelConfig config) : config_(config) {}

auto UnicycleModel::propagate(const MotionState &state, const types::Twist &command,
                              double deltaSeconds) const -> Result<MotionResult> {
  if (deltaSeconds <= 0.0 || !std::isfinite(deltaSeconds)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Delta time must be positive."});
  }

  if (!std::isfinite(command.v) || !std::isfinite(command.w)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Command velocities must be finite."});
  }

  if (!std::isfinite(state.twist.v) || !std::isfinite(state.twist.w)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Current velocities must be finite."});
  }

  if (config_.maxLinearSpeed <= 0.0 || config_.maxAngularSpeed <= 0.0 ||
      config_.maxLinearAcceleration <= 0.0 || config_.maxAngularAcceleration <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Model speed and acceleration limits must be positive."});
  }
  if (config_.tauLinear < 0.0 || config_.tauAngular < 0.0 || !std::isfinite(config_.tauLinear) ||
      !std::isfinite(config_.tauAngular)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Model lag constants are invalid."});
  }

  const auto speedLimitedCommand =
      types::Twist{.v = std::clamp(command.v, -config_.maxLinearSpeed, config_.maxLinearSpeed),
                   .w = std::clamp(command.w, -config_.maxAngularSpeed, config_.maxAngularSpeed)};

  const auto maxLinearStepDelta = config_.maxLinearAcceleration * deltaSeconds;
  const auto maxAngularStepDelta = config_.maxAngularAcceleration * deltaSeconds;

  const auto linearAlpha =
      config_.tauLinear == 0.0 ? 1.0 : (deltaSeconds / (config_.tauLinear + deltaSeconds));
  const auto angularAlpha =
      config_.tauAngular == 0.0 ? 1.0 : (deltaSeconds / (config_.tauAngular + deltaSeconds));

  const auto laggedLinearTarget =
      state.twist.v + (linearAlpha * (speedLimitedCommand.v - state.twist.v));
  const auto laggedAngularTarget =
      state.twist.w + (angularAlpha * (speedLimitedCommand.w - state.twist.w));

  const auto appliedLinearVelocity =
      state.twist.v +
      std::clamp(laggedLinearTarget - state.twist.v, -maxLinearStepDelta, maxLinearStepDelta);
  const auto appliedAngularVelocity =
      state.twist.w +
      std::clamp(laggedAngularTarget - state.twist.w, -maxAngularStepDelta, maxAngularStepDelta);

  const auto saturatedCommand = types::Twist{
      .v = std::clamp(appliedLinearVelocity, -config_.maxLinearSpeed, config_.maxLinearSpeed),
      .w = std::clamp(appliedAngularVelocity, -config_.maxAngularSpeed, config_.maxAngularSpeed)};

  const auto deltaX = saturatedCommand.v * std::cos(state.pose.theta) * deltaSeconds;
  const auto deltaY = saturatedCommand.v * std::sin(state.pose.theta) * deltaSeconds;
  const auto deltaTheta = saturatedCommand.w * deltaSeconds;

  const types::Pose nextPose{.x = state.pose.x + deltaX,
                             .y = state.pose.y + deltaY,
                             .theta = state.pose.theta + deltaTheta};

  return MotionResult{.pose = nextPose, .twist = saturatedCommand};
}

} // namespace ad::simulation
