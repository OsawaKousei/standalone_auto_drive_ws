#include "unicycle_model.hpp"

#include <cmath>

namespace ad::simulation {

auto UnicycleModel::propagate(const MotionState &state, const types::Twist &command,
                              double deltaSeconds) const -> Result<MotionResult> {
  if (deltaSeconds <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "deltaSeconds must be positive."});
  }

  const auto dx = command.v * std::cos(state.pose.theta) * deltaSeconds;
  const auto dy = command.v * std::sin(state.pose.theta) * deltaSeconds;
  const auto dtheta = command.w * deltaSeconds;

  const types::Pose nextPose{state.pose.x + dx, state.pose.y + dy, state.pose.theta + dtheta};
  const types::Twist nextTwist{command.v, command.w};

  return MotionResult{nextPose, nextTwist};
}

} // namespace ad::simulation
