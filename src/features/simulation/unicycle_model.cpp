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

  if (config_.maxLinearSpeed <= 0.0 || config_.maxAngularSpeed <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Model speed limits must be positive."});
  }

  const auto saturatedCommand =
      types::Twist{.v = std::clamp(command.v, -config_.maxLinearSpeed, config_.maxLinearSpeed),
                   .w = std::clamp(command.w, -config_.maxAngularSpeed, config_.maxAngularSpeed)};

  const auto deltaX = saturatedCommand.v * std::cos(state.pose.theta) * deltaSeconds;
  const auto deltaY = saturatedCommand.v * std::sin(state.pose.theta) * deltaSeconds;
  const auto deltaTheta = saturatedCommand.w * deltaSeconds;

  const types::Pose nextPose{.x = state.pose.x + deltaX,
                             .y = state.pose.y + deltaY,
                             .theta = state.pose.theta + deltaTheta};

  return MotionResult{.pose = nextPose, .twist = saturatedCommand};
}

} // namespace ad::simulation
