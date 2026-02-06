#include "visualizer.hpp"

#include <fmt/core.h>

namespace ad::visualization {

auto Visualizer::renderPath(std::span<const types::Point> path) const -> Status {
  if (path.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Path is empty."});
  }
  fmt::print("Render path with {} waypoints\n", path.size());
  return {};
}

auto Visualizer::renderPose(const types::Pose &pose) const -> Status {
  fmt::print("Render pose -> x: {:.2f}, y: {:.2f}, theta: {:.2f}\n", pose.x, pose.y, pose.theta);
  return {};
}

} // namespace ad::visualization
