#include "localizer_util.hpp"

#include <cmath>
#include <numbers>
#include <ranges>

namespace ad::localization::util {

auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
  if (angle < 0.0) {
    angle += 2.0 * std::numbers::pi;
  }
  return angle - std::numbers::pi;
}

auto mapHasConsistentGrid(const types::MapData &map) -> bool {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.grid.size() == expectedCells;
}

auto collectOccupiedPoints(const types::MapData &map) -> std::vector<types::Point> {
  const auto width = static_cast<std::size_t>(map.width);
  const auto height = static_cast<std::size_t>(map.height);
  auto points = std::vector<types::Point>{};

  for (const auto rowIndex : std::views::iota(std::size_t{0}, height)) {
    for (const auto colIndex : std::views::iota(std::size_t{0}, width)) {
      const auto index = (rowIndex * width) + colIndex;
      if (map.grid[index] <= 0) {
        continue;
      }

      const auto xValue = (static_cast<double>(colIndex) + 0.5) * map.resolution;
      const auto yValue = (static_cast<double>(rowIndex) + 0.5) * map.resolution;
      points.push_back(types::Point{.x = xValue, .y = yValue});
    }
  }

  return points;
}

} // namespace ad::localization::util
