#include "lidar_sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>

namespace {

[[nodiscard]] auto mapHasConsistentGrid(const ad::types::MapData &map) -> bool {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.grid.size() == expectedCells;
}

[[nodiscard]] auto cellIndex(const ad::types::MapData &map, double x, double y)
    -> std::optional<std::size_t> {
  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return std::nullopt;
  }

  const auto column = static_cast<int>(std::floor(x / map.resolution));
  const auto row = static_cast<int>(std::floor(y / map.resolution));

  const bool inBounds = column >= 0 && column < map.width && row >= 0 && row < map.height;
  if (!inBounds) {
    return std::nullopt;
  }

  const auto index = static_cast<std::size_t>(row * map.width + column);
  return index;
}

} // namespace

namespace ad::simulation {

auto LidarSim::simulate(const types::MapData &map, const types::Pose &pose) const
    -> Result<LidarScan> {
  if (!mapHasConsistentGrid(map)) {
    return tl::make_unexpected(Error{.code = ErrorCode::SizeMismatch,
                                     .message = "Map grid size does not match width and height."});
  }

  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Map dimensions must be positive."});
  }

  const auto rayCount = static_cast<std::size_t>(std::max(map.width, 1));
  const auto maxRange = map.resolution * static_cast<double>(std::max(map.width, map.height));
  const auto stepLimit = static_cast<std::size_t>(std::ceil(maxRange / map.resolution));
  const auto angleStep = (2.0 * std::numbers::pi) / static_cast<double>(rayCount);

  const auto traceRay = [&](double angle) -> double {
    const auto steps = std::views::iota(std::size_t{1}, stepLimit + 1);
    const auto hit = std::ranges::find_if(steps, [&](std::size_t stepIndex) -> bool {
      const auto distance = map.resolution * static_cast<double>(stepIndex);
      const auto x = pose.x + std::cos(angle) * distance;
      const auto y = pose.y + std::sin(angle) * distance;
      const auto index = cellIndex(map, x, y);
      if (!index.has_value()) {
        return true;
      }
      return map.grid[*index] > 0;
    });

    if (hit == std::ranges::end(steps)) {
      return maxRange;
    }

    const auto distance = map.resolution * static_cast<double>(*hit);
    return std::clamp(distance, 0.0, maxRange);
  };

  auto scan = LidarScan{};
  scan.reserve(rayCount);

  const auto rayIndices = std::views::iota(std::size_t{0}, rayCount);
  std::ranges::transform(rayIndices, std::back_inserter(scan), [&](std::size_t index) -> double {
    const auto angle = pose.theta - std::numbers::pi + (angleStep * static_cast<double>(index));
    return traceRay(angle);
  });

  return scan;
}

} // namespace ad::simulation
