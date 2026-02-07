#include "pure_pursuit.hpp"

#include <cmath>
#include <ranges>

namespace ad::control {

PurePursuitController::PurePursuitController(PurePursuitConfig config) : config_(config) {}

auto PurePursuitController::computeCommand(const ControlInput &input) const
    -> Result<types::Twist> {
  if (input.path.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Path is empty."});
  }
  if (config_.lookaheadDistance <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Lookahead must be positive."});
  }
  if (config_.desiredLinearVelocity < 0.0) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Desired linear velocity must be non-negative."});
  }

  const auto &pose = input.currentPose;
  const auto targetIt = std::ranges::find_if(input.path, [&](const types::Point &point) {
    const auto dx = point.x - pose.x;
    const auto dy = point.y - pose.y;
    return std::hypot(dx, dy) >= config_.lookaheadDistance;
  });
  const auto target = (targetIt == input.path.end()) ? input.path.back() : *targetIt;
  const auto dx = target.x - pose.x;
  const auto dy = target.y - pose.y;
  const auto distance = std::hypot(dx, dy);
  if (distance <= 0.0) {
    return types::Twist{0.0, 0.0};
  }

  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);
  const auto xLocal = cosTheta * dx + sinTheta * dy;
  const auto yLocal = -sinTheta * dx + cosTheta * dy;
  if (xLocal <= 0.0) {
    return types::Twist{0.0, 0.0};
  }

  const auto curvature = (2.0 * yLocal) / (distance * distance);
  const auto linearVelocity = config_.desiredLinearVelocity;
  const auto angularVelocity = curvature * linearVelocity;

  return types::Twist{linearVelocity, angularVelocity};
}

} // namespace ad::control
