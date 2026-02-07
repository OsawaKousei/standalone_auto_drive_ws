#include "dijkstra_planner.hpp"
#include "planner_utils.hpp"

#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <ranges>
#include <utility>
#include <vector>

namespace ad::planning {

namespace {

struct Move {
  const int dx;
  const int dy;
  const double cost;
};

struct CurrentNode {
  const utils::GridCoord coord;
  const std::size_t index;
  const double cost;
};

struct StartGoalInfo {
  const std::size_t startIndex;
  const std::size_t goalIndex;
  const types::Point startCenter;
};

[[nodiscard]] auto inBounds(const types::MapData &map, const utils::GridCoord &coord) -> bool {
  return coord.x >= 0 && coord.y >= 0 && coord.x < map.width && coord.y < map.height;
}

[[nodiscard]] auto isDiagonal(const Move &move) -> bool { return move.dx != 0 && move.dy != 0; }

[[nodiscard]] auto canMoveDiagonal(const types::MapData &map, const utils::GridCoord &from,
                                   const Move &move) -> bool {
  if (!isDiagonal(move)) {
    return true;
  }

  const auto sideX = utils::GridCoord{.x = from.x + move.dx, .y = from.y};
  const auto sideY = utils::GridCoord{.x = from.x, .y = from.y + move.dy};
  return !utils::isObstacle(map, sideX) && !utils::isObstacle(map, sideY);
}

using Node = std::pair<double, std::size_t>;
using Frontier = std::priority_queue<Node, std::vector<Node>, std::greater<>>;

auto tryRelaxNeighbor(const types::MapData &map, const CurrentNode &current, const Move &move,
                      std::vector<double> &distances,
                      std::vector<std::optional<std::size_t>> &previous, Frontier &frontier)
    -> void {
  const auto neighbor =
      utils::GridCoord{.x = current.coord.x + move.dx, .y = current.coord.y + move.dy};
  if (!inBounds(map, neighbor)) {
    return;
  }

  if (!canMoveDiagonal(map, current.coord, move)) {
    return;
  }

  if (utils::isObstacle(map, neighbor)) {
    return;
  }

  const auto neighborIndex = utils::toIndex(map, neighbor);
  const auto nextCost = current.cost + move.cost;
  if (nextCost >= distances[neighborIndex]) {
    return;
  }

  distances[neighborIndex] = nextCost;
  previous[neighborIndex] = current.index;
  frontier.emplace(nextCost, neighborIndex);
}

[[nodiscard]] auto validateInputs(const types::MapData &map, const types::Pose &start,
                                  const types::Pose &goal) -> Result<StartGoalInfo> {
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

[[nodiscard]] auto computePrevious(const types::MapData &map, const StartGoalInfo &startGoal)
    -> Result<std::vector<std::optional<std::size_t>>> {
  const auto totalCells = utils::cellCount(map);
  auto previous = std::vector<std::optional<std::size_t>>(totalCells, std::nullopt);
  auto distances = std::vector<double>(totalCells, std::numeric_limits<double>::infinity());

  auto frontier = Frontier{};
  distances[startGoal.startIndex] = 0.0;
  frontier.emplace(0.0, startGoal.startIndex);

  const auto sqrt2 = std::numbers::sqrt2;
  const auto moves = std::vector<Move>{
      Move{.dx = 1, .dy = 0, .cost = 1.0},    Move{.dx = -1, .dy = 0, .cost = 1.0},
      Move{.dx = 0, .dy = 1, .cost = 1.0},    Move{.dx = 0, .dy = -1, .cost = 1.0},
      Move{.dx = 1, .dy = 1, .cost = sqrt2},  Move{.dx = 1, .dy = -1, .cost = sqrt2},
      Move{.dx = -1, .dy = 1, .cost = sqrt2}, Move{.dx = -1, .dy = -1, .cost = sqrt2}};

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
    const auto currentNode = CurrentNode{.coord = coord, .index = current, .cost = currentCost};
    for (const auto &move : moves) {
      tryRelaxNeighbor(map, currentNode, move, distances, previous, frontier);
    }
  }

  if (!previous[startGoal.goalIndex]) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "No path found to the goal."});
  }

  return previous;
}

[[nodiscard]] auto buildPath(const types::MapData &map,
                             const std::vector<std::optional<std::size_t>> &previous,
                             const StartGoalInfo &startGoal) -> types::Path {
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

} // namespace

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

} // namespace ad::planning
