#include "pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace ad::control {

auto PurePursuitController::computeCommand(std::span<const types::Point> path,
                                           const types::Pose &currentPose,
                                           double lookaheadDistance) const -> Result<types::Twist> {
  if (path.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Path is empty."});
  }
  if (lookaheadDistance <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Lookahead must be positive."});
  }

  const auto target = path.front();
  const auto dx = target.x - currentPose.x;
  const auto dy = target.y - currentPose.y;
  const auto distance = std::hypot(dx, dy);
  if (distance < lookaheadDistance) {
    return types::Twist{0.0, 0.0};
  }

  const auto heading = std::atan2(dy, dx);
  const auto headingError = heading - currentPose.theta;
  const auto angularVelocity = headingError / lookaheadDistance;
  const auto linearVelocity = std::max(0.0, distance / lookaheadDistance);

  return types::Twist{linearVelocity, angularVelocity};
}

} // namespace ad::control
