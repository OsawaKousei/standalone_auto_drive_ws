#include "planner_utils.hpp"

#include <cmath>

namespace ad::planning::utils {

[[nodiscard]] auto cellCount(const types::MapData &map) -> std::size_t {
  return static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
}

[[nodiscard]] auto isValidMap(const types::MapData &map) -> Result<void> {
  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Map has invalid dimensions."});
  }

  const auto expectedCells = cellCount(map);
  if (map.grid.size() != expectedCells) {
    return tl::make_unexpected(Error{.code = ErrorCode::SizeMismatch,
                                     .message = "Map grid size does not match width and height."});
  }

  return {};
}

[[nodiscard]] auto toIndex(const types::MapData &map, const GridCoord &coord) -> std::size_t {
  const auto width = static_cast<std::size_t>(map.width);
  return (static_cast<std::size_t>(coord.y) * width) + static_cast<std::size_t>(coord.x);
}

[[nodiscard]] auto toCoord(const types::MapData &map, std::size_t index) -> GridCoord {
  const auto width = static_cast<std::size_t>(map.width);
  const auto xValue = static_cast<int>(index % width);
  const auto yValue = static_cast<int>(index / width);
  return GridCoord{.x = xValue, .y = yValue};
}

[[nodiscard]] auto cellCenter(const types::MapData &map, const GridCoord &coord) -> types::Point {
  const auto half = 0.5 * map.resolution;
  return types::Point{.x = (static_cast<double>(coord.x) * map.resolution) + half,
                      .y = (static_cast<double>(coord.y) * map.resolution) + half};
}

[[nodiscard]] auto worldToCell(const types::MapData &map, const types::Pose &pose)
    -> Result<GridCoord> {
  if (pose.x < 0.0 || pose.y < 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Pose is outside the map bounds."});
  }

  const auto cellX = static_cast<int>(std::floor(pose.x / map.resolution));
  const auto cellY = static_cast<int>(std::floor(pose.y / map.resolution));
  if (cellX < 0 || cellY < 0 || cellX >= map.width || cellY >= map.height) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Pose is outside the map bounds."});
  }

  return GridCoord{.x = cellX, .y = cellY};
}

[[nodiscard]] auto isObstacle(const types::MapData &map, const GridCoord &coord) -> bool {
  const auto index = toIndex(map, coord);
  return map.grid[index] > 0;
}

} // namespace ad::planning::utils
