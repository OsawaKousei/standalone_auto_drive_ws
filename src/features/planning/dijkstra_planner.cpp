#include "dijkstra_planner.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <ranges>
#include <vector>

namespace ad::planning {
namespace {

struct GridCoord {
  const int x;
  const int y;
};

struct GridOffset {
  const int dx;
  const int dy;
};

struct FrontierNode {
  double cost;
  std::size_t index;
};

struct FrontierCompare {
  bool operator()(const FrontierNode &left, const FrontierNode &right) const {
    return left.cost > right.cost;
  }
};

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
  const auto x = static_cast<int>(index % width);
  const auto y = static_cast<int>(index / width);
  return GridCoord{.x = x, .y = y};
}

[[nodiscard]] auto cellCenter(const types::MapData &map, const GridCoord &coord) -> types::Point {
  const auto half = 0.5 * map.resolution;
  return types::Point{(static_cast<double>(coord.x) * map.resolution) + half,
                      (static_cast<double>(coord.y) * map.resolution) + half};
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

} // namespace

auto DijkstraPlanner::plan(const types::MapData &map, const types::Pose &start,
                           const types::Pose &goal) const -> Result<types::Path> {
  const auto mapStatus = isValidMap(map);
  if (!mapStatus) {
    return tl::make_unexpected(mapStatus.error());
  }

  const auto startCell = worldToCell(map, start);
  if (!startCell) {
    return tl::make_unexpected(startCell.error());
  }

  const auto goalCell = worldToCell(map, goal);
  if (!goalCell) {
    return tl::make_unexpected(goalCell.error());
  }

  if (isObstacle(map, *startCell) || isObstacle(map, *goalCell)) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Start or goal cell is occupied by an obstacle."});
  }

  const auto startIndex = toIndex(map, *startCell);
  const auto goalIndex = toIndex(map, *goalCell);
  if (startIndex == goalIndex) {
    return types::Path{cellCenter(map, *startCell)};
  }

  const auto totalCells = cellCount(map);
  auto distances = std::vector<double>(totalCells, std::numeric_limits<double>::infinity());
  auto previous = std::vector<std::optional<std::size_t>>(totalCells, std::nullopt);

  std::priority_queue<FrontierNode, std::vector<FrontierNode>, FrontierCompare> frontier;
  distances[startIndex] = 0.0;
  frontier.push(FrontierNode{.cost = 0.0, .index = startIndex});

  const auto offsets = std::vector<GridOffset>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!frontier.empty()) {
    const auto current = frontier.top();
    frontier.pop();

    if (current.cost > distances[current.index]) {
      continue;
    }

    if (current.index == goalIndex) {
      break;
    }

    const auto coord = toCoord(map, current.index);
    for (const auto &offset : offsets) {
      const auto neighbor = GridCoord{.x = coord.x + offset.dx, .y = coord.y + offset.dy};
      if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= map.width || neighbor.y >= map.height) {
        continue;
      }

      if (isObstacle(map, neighbor)) {
        continue;
      }

      const auto neighborIndex = toIndex(map, neighbor);
      const auto tentativeCost = current.cost + 1.0;
      if (tentativeCost < distances[neighborIndex]) {
        distances[neighborIndex] = tentativeCost;
        previous[neighborIndex] = current.index;
        frontier.push(FrontierNode{.cost = tentativeCost, .index = neighborIndex});
      }
    }
  }

  if (!previous[goalIndex]) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "No path found to the goal."});
  }

  const auto path = [&]() {
    auto reversedIndices = std::vector<std::size_t>{};
    auto current = std::optional<std::size_t>{goalIndex};
    while (current) {
      reversedIndices.push_back(*current);
      if (*current == startIndex) {
        break;
      }
      current = previous[*current];
    }

    auto forward = std::vector<types::Point>{};
    forward.reserve(reversedIndices.size());
    for (const auto index : std::views::reverse(reversedIndices)) {
      forward.push_back(cellCenter(map, toCoord(map, index)));
    }
    return forward;
  }();

  return path;
}

} // namespace ad::planning
