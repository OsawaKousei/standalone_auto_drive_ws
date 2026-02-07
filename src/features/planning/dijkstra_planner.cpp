#include "dijkstra_planner.hpp"
#include "planner_utils.hpp"

#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <ranges>
#include <utility>
#include <vector>

namespace ad::planning {

auto DijkstraPlanner::plan(const types::MapData &map, const types::Pose &start,
                           const types::Pose &goal) const -> Result<types::Path> {
  const auto mapStatus = utils::isValidMap(map);
  if (!mapStatus) {
    return tl::make_unexpected(mapStatus.error());
  }

  const auto startCell = utils::worldToCell(map, start);
  if (!startCell) {
    return tl::make_unexpected(startCell.error());
  }

  const auto goalCell = utils::worldToCell(map, goal);
  if (!goalCell) {
    return tl::make_unexpected(goalCell.error());
  }

  if (utils::isObstacle(map, *startCell) || utils::isObstacle(map, *goalCell)) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Start or goal cell is occupied by an obstacle."});
  }

  const auto startIndex = utils::toIndex(map, *startCell);
  const auto goalIndex = utils::toIndex(map, *goalCell);
  if (startIndex == goalIndex) {
    return types::Path{utils::cellCenter(map, *startCell)};
  }

  const auto totalCells = utils::cellCount(map);
  auto previous = std::vector<std::optional<std::size_t>>(totalCells, std::nullopt);
  auto distances = std::vector<double>(totalCells, std::numeric_limits<double>::infinity());

  using Node = std::pair<double, std::size_t>;
  auto frontier = std::priority_queue<Node, std::vector<Node>, std::greater<>>{};
  distances[startIndex] = 0.0;
  frontier.emplace(0.0, startIndex);

  const auto offsets = std::vector<utils::GridOffset>{
      utils::GridOffset{.dx = 1, .dy = 0}, utils::GridOffset{.dx = -1, .dy = 0},
      utils::GridOffset{.dx = 0, .dy = 1}, utils::GridOffset{.dx = 0, .dy = -1}};

  while (!frontier.empty()) {
    const auto [currentCost, current] = frontier.top();
    frontier.pop();

    if (currentCost > distances[current]) {
      continue;
    }

    if (current == goalIndex) {
      break;
    }

    const auto coord = utils::toCoord(map, current);
    for (const auto &offset : offsets) {
      const auto neighbor = utils::GridCoord{.x = coord.x + offset.dx, .y = coord.y + offset.dy};
      if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= map.width || neighbor.y >= map.height) {
        continue;
      }

      if (utils::isObstacle(map, neighbor)) {
        continue;
      }

      const auto neighborIndex = utils::toIndex(map, neighbor);
      const auto nextCost = currentCost + 1.0;
      if (nextCost >= distances[neighborIndex]) {
        continue;
      }

      distances[neighborIndex] = nextCost;
      previous[neighborIndex] = current;
      frontier.emplace(nextCost, neighborIndex);
    }
  }

  if (!previous[goalIndex] && startIndex != goalIndex) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "No path found to the goal."});
  }

  const auto path = [&]() -> types::Path {
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
      forward.push_back(utils::cellCenter(map, utils::toCoord(map, index)));
    }
    return forward;
  }();

  return path;
}

} // namespace ad::planning
