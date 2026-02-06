#include "visualizer.hpp"

#include <algorithm>
#include <fmt/core.h>
#include <ranges>

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

auto Visualizer::renderScan(std::span<const double> ranges) const -> Status {
  if (ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan is empty."});
  }

  const auto [minIt, maxIt] = std::ranges::minmax_element(ranges);
  fmt::print("Render scan ({} rays) -> min: {:.2f} m, max: {:.2f} m\n", ranges.size(), *minIt,
             *maxIt);
  return {};
}

} // namespace ad::visualization
