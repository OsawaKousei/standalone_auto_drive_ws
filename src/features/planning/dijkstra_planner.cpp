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
  const auto startGoal = validateInputs(map, start, goal);
  if (!startGoal) {
    return tl::make_unexpected(startGoal.error());
  }

  if (startGoal->startIndex == startGoal->goalIndex) {
    return types::Path{startGoal->startCenter};
  }

  const auto previous = computePrevious(map, *startGoal);
  if (!previous) {
    return tl::make_unexpected(previous.error());
  }

  return buildPath(map, *previous, *startGoal);
}

auto DijkstraPlanner::validateInputs(const types::MapData &map, const types::Pose &start,
                                     const types::Pose &goal) const -> Result<StartGoalInfo> {
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

  return StartGoalInfo{.startIndex = utils::toIndex(map, *startCell),
                       .goalIndex = utils::toIndex(map, *goalCell),
                       .startCenter = utils::cellCenter(map, *startCell)};
}

auto DijkstraPlanner::computePrevious(const types::MapData &map,
                                      const StartGoalInfo &startGoal) const
    -> Result<std::vector<std::optional<std::size_t>>> {
  const auto totalCells = utils::cellCount(map);
  auto previous = std::vector<std::optional<std::size_t>>(totalCells, std::nullopt);
  auto distances = std::vector<double>(totalCells, std::numeric_limits<double>::infinity());

  using Node = std::pair<double, std::size_t>;
  auto frontier = std::priority_queue<Node, std::vector<Node>, std::greater<>>{};
  distances[startGoal.startIndex] = 0.0;
  frontier.emplace(0.0, startGoal.startIndex);

  const auto offsets = std::vector<utils::GridOffset>{
      utils::GridOffset{.dx = 1, .dy = 0}, utils::GridOffset{.dx = -1, .dy = 0},
      utils::GridOffset{.dx = 0, .dy = 1}, utils::GridOffset{.dx = 0, .dy = -1}};

  while (!frontier.empty()) {
    const auto [currentCost, current] = frontier.top();
    frontier.pop();

    if (currentCost > distances[current]) {
      continue;
    }

    if (current == startGoal.goalIndex) {
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

  if (!previous[startGoal.goalIndex]) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "No path found to the goal."});
  }

  return previous;
}

auto DijkstraPlanner::buildPath(const types::MapData &map,
                                const std::vector<std::optional<std::size_t>> &previous,
                                const StartGoalInfo &startGoal) const -> types::Path {
  auto reversedIndices = std::vector<std::size_t>{};
  auto current = std::optional<std::size_t>{startGoal.goalIndex};
  while (current) {
    reversedIndices.push_back(*current);
    if (*current == startGoal.startIndex) {
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
}

} // namespace ad::planning
