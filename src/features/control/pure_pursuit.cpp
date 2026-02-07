#include "pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>

namespace ad::control {

PurePursuitController::PurePursuitController(PurePursuitConfig config) : config_(config) {}

auto PurePursuitController::computeCommand(const ControlInput &input) const
    -> Result<types::Twist> {
  if (input.path.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Path is empty."});
  }
  if (config_.lookaheadDistance <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Lookahead must be positive."});
  }
  if (config_.desiredLinearVelocity < 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Desired linear velocity must be non-negative."});
  }

  const auto &pose = input.currentPose;
  const auto target = [&]() -> types::Point {
    if (input.path.size() == 1U) {
      return input.path.front();
    }

    const auto indices = std::views::iota(std::size_t{0}, input.path.size());
    const auto closestIndex =
        *std::ranges::min_element(indices, [&](std::size_t lhs, std::size_t rhs) -> bool {
          const auto dxLeft = input.path[lhs].x - pose.x;
          const auto dyLeft = input.path[lhs].y - pose.y;
          const auto dxRight = input.path[rhs].x - pose.x;
          const auto dyRight = input.path[rhs].y - pose.y;
          return std::hypot(dxLeft, dyLeft) < std::hypot(dxRight, dyRight);
        });

    auto remaining = config_.lookaheadDistance;
    auto currentX = input.path[closestIndex].x;
    auto currentY = input.path[closestIndex].y;
    const auto lastIndex = input.path.size() - 1U;
    for (const auto index : std::views::iota(closestIndex, lastIndex)) {
      const auto next = input.path[index + 1U];
      const auto segment = std::hypot(next.x - currentX, next.y - currentY);
      if (segment <= 0.0) {
        currentX = next.x;
        currentY = next.y;
        continue;
      }
      if (remaining <= segment) {
        const auto interpolationRatio = remaining / segment;
        return types::Point{.x = currentX + (interpolationRatio * (next.x - currentX)),
                            .y = currentY + (interpolationRatio * (next.y - currentY))};
      }
      remaining -= segment;
      currentX = next.x;
      currentY = next.y;
    }
    return input.path.back();
  }();

  const auto deltaX = target.x - pose.x;
  const auto deltaY = target.y - pose.y;
  const auto distance = std::hypot(deltaX, deltaY);
  if (distance <= 0.0) {
    return types::Twist{.v = 0.0, .w = 0.0};
  }

  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);
  const auto xLocal = (cosTheta * deltaX) + (sinTheta * deltaY);
  const auto yLocal = (-sinTheta * deltaX) + (cosTheta * deltaY);

  const auto curvature = (2.0 * yLocal) / (distance * distance);
  const auto linearVelocity = config_.desiredLinearVelocity;
  const auto angularVelocity = curvature * linearVelocity;

  if (xLocal < 0.0) {
    return types::Twist{.v = -linearVelocity, .w = angularVelocity};
  }

  return types::Twist{.v = linearVelocity, .w = angularVelocity};
}

} // namespace ad::control
