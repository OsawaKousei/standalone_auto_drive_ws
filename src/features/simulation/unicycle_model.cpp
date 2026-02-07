#include "unicycle_model.hpp"

#include <algorithm>
#include <cmath>

namespace ad::simulation {

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

  constexpr double kMaxLinearSpeed = 5.0;
  constexpr double kMaxAngularSpeed = 3.0;

  const auto saturatedCommand =
      types::Twist{.v = std::clamp(command.v, -kMaxLinearSpeed, kMaxLinearSpeed),
                   .w = std::clamp(command.w, -kMaxAngularSpeed, kMaxAngularSpeed)};

  const auto dx = saturatedCommand.v * std::cos(state.pose.theta) * deltaSeconds;
  const auto dy = saturatedCommand.v * std::sin(state.pose.theta) * deltaSeconds;
  const auto dtheta = saturatedCommand.w * deltaSeconds;

  const types::Pose nextPose{
      .x = state.pose.x + dx, .y = state.pose.y + dy, .theta = state.pose.theta + dtheta};

  return MotionResult{.pose = nextPose, .twist = saturatedCommand};
}

} // namespace ad::simulation
