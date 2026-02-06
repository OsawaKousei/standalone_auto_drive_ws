#include "unicycle_model.hpp"

#include <algorithm>
#include <cmath>

namespace ad::simulation {

auto UnicycleModel::propagate(const MotionState &state, const types::Twist &command,
                              double deltaSeconds) const -> Result<MotionResult> {
  if (deltaSeconds <= 0.0 || !std::isfinite(deltaSeconds)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "deltaSeconds must be finite and positive."});
  }

  if (!std::isfinite(command.v) || !std::isfinite(command.w)) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Command must be finite."});
  }

  constexpr double kMaxLinearSpeed = 5.0;
  constexpr double kMaxAngularSpeed = 3.0;

  const auto saturatedCommand =
      types::Twist{std::clamp(command.v, -kMaxLinearSpeed, kMaxLinearSpeed),
                   std::clamp(command.w, -kMaxAngularSpeed, kMaxAngularSpeed)};

  const auto dx = saturatedCommand.v * std::cos(state.pose.theta) * deltaSeconds;
  const auto dy = saturatedCommand.v * std::sin(state.pose.theta) * deltaSeconds;
  const auto dtheta = saturatedCommand.w * deltaSeconds;

  const types::Pose nextPose{state.pose.x + dx, state.pose.y + dy, state.pose.theta + dtheta};

  return MotionResult{nextPose, saturatedCommand};
}

} // namespace ad::simulation
