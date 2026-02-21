#include "holonomic_model.hpp"

#include <algorithm>
#include <cmath>

namespace ad::simulation {

HolonomicModel::HolonomicModel(HolonomicModelConfig config) : config_(config) {}

auto HolonomicModel::propagate(const MotionState &state, const types::Twist &command,
                               double deltaSeconds) const -> Result<MotionResult> {
  if (deltaSeconds <= 0.0 || !std::isfinite(deltaSeconds)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Delta time must be positive."});
  }

  if (!std::isfinite(command.v) || !std::isfinite(command.vy) || !std::isfinite(command.w)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Command velocities must be finite."});
  }

  if (!std::isfinite(state.twist.v) || !std::isfinite(state.twist.vy) ||
      !std::isfinite(state.twist.w)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Current velocities must be finite."});
  }

  if (config_.maxLinearSpeedX <= 0.0 || config_.maxLinearSpeedY <= 0.0 ||
      config_.maxAngularSpeed <= 0.0 || config_.maxLinearAccelerationX <= 0.0 ||
      config_.maxLinearAccelerationY <= 0.0 || config_.maxAngularAcceleration <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Model speed and acceleration limits must be positive."});
  }

  if (config_.tauLinearX < 0.0 || config_.tauLinearY < 0.0 || config_.tauAngular < 0.0 ||
      !std::isfinite(config_.tauLinearX) || !std::isfinite(config_.tauLinearY) ||
      !std::isfinite(config_.tauAngular)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Model lag constants are invalid."});
  }

  const auto limitedCommand =
      types::Twist{.v = std::clamp(command.v, -config_.maxLinearSpeedX, config_.maxLinearSpeedX),
                   .vy = std::clamp(command.vy, -config_.maxLinearSpeedY, config_.maxLinearSpeedY),
                   .w = std::clamp(command.w, -config_.maxAngularSpeed, config_.maxAngularSpeed)};

  const auto linearXAlpha =
      config_.tauLinearX == 0.0 ? 1.0 : (deltaSeconds / (config_.tauLinearX + deltaSeconds));
  const auto linearYAlpha =
      config_.tauLinearY == 0.0 ? 1.0 : (deltaSeconds / (config_.tauLinearY + deltaSeconds));
  const auto angularAlpha =
      config_.tauAngular == 0.0 ? 1.0 : (deltaSeconds / (config_.tauAngular + deltaSeconds));

  const auto laggedTargetVx = state.twist.v + (linearXAlpha * (limitedCommand.v - state.twist.v));
  const auto laggedTargetVy =
      state.twist.vy + (linearYAlpha * (limitedCommand.vy - state.twist.vy));
  const auto laggedTargetW = state.twist.w + (angularAlpha * (limitedCommand.w - state.twist.w));

  const auto maxLinearXDelta = config_.maxLinearAccelerationX * deltaSeconds;
  const auto maxLinearYDelta = config_.maxLinearAccelerationY * deltaSeconds;
  const auto maxAngularDelta = config_.maxAngularAcceleration * deltaSeconds;

  const auto appliedVx =
      state.twist.v + std::clamp(laggedTargetVx - state.twist.v, -maxLinearXDelta, maxLinearXDelta);
  const auto appliedVy = state.twist.vy + std::clamp(laggedTargetVy - state.twist.vy,
                                                     -maxLinearYDelta, maxLinearYDelta);
  const auto appliedW =
      state.twist.w + std::clamp(laggedTargetW - state.twist.w, -maxAngularDelta, maxAngularDelta);

  const auto saturatedCommand =
      types::Twist{.v = std::clamp(appliedVx, -config_.maxLinearSpeedX, config_.maxLinearSpeedX),
                   .vy = std::clamp(appliedVy, -config_.maxLinearSpeedY, config_.maxLinearSpeedY),
                   .w = std::clamp(appliedW, -config_.maxAngularSpeed, config_.maxAngularSpeed)};

  const auto cosTheta = std::cos(state.pose.theta);
  const auto sinTheta = std::sin(state.pose.theta);

  const auto deltaX =
      ((saturatedCommand.v * cosTheta) - (saturatedCommand.vy * sinTheta)) * deltaSeconds;
  const auto deltaY =
      ((saturatedCommand.v * sinTheta) + (saturatedCommand.vy * cosTheta)) * deltaSeconds;
  const auto deltaTheta = saturatedCommand.w * deltaSeconds;

  const types::Pose nextPose{.x = state.pose.x + deltaX,
                             .y = state.pose.y + deltaY,
                             .theta = state.pose.theta + deltaTheta};

  return MotionResult{.pose = nextPose, .twist = saturatedCommand};
}

} // namespace ad::simulation
