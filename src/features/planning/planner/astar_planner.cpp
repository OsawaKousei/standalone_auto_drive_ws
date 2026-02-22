#include "astar_planner.hpp"
#include "../planner_utils.hpp"

#include <algorithm>
#include <cmath>
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
  const utils::GridCoord startCoord;
  const utils::GridCoord goalCoord;
  const types::Point startCenter;
};

[[nodiscard]] auto inBounds(const types::MapData &map, const utils::GridCoord &coord) -> bool {
  return coord.x >= 0 && coord.y >= 0 && coord.x < map.width && coord.y < map.height;
}

[[nodiscard]] auto isDiagonal(const Move &move) -> bool { return move.dx != 0 && move.dy != 0; }

[[nodiscard]] auto cellCenterPose(const types::MapData &map, const utils::GridCoord &coord)
    -> types::Pose {
  const auto center = utils::cellCenter(map, coord);
  return types::Pose{.x = center.x, .y = center.y, .theta = 0.0};
}

[[nodiscard]] auto isFreeCell(const types::MapData &map, const ICollisionChecker &collisionChecker,
                              const types::Footprint &footprint, const utils::GridCoord &coord)
    -> Result<bool> {
  const auto pose = cellCenterPose(map, coord);
  return collisionChecker.isFree(pose, footprint);
}

[[nodiscard]] auto canMoveDiagonal(const types::MapData &map, const ICollisionChecker &checker,
                                   const types::Footprint &footprint, const utils::GridCoord &from,
                                   const Move &move) -> Result<bool> {
  if (!isDiagonal(move)) {
    return true;
  }

  const auto sideX = utils::GridCoord{.x = from.x + move.dx, .y = from.y};
  const auto sideY = utils::GridCoord{.x = from.x, .y = from.y + move.dy};
  const auto sideXFree = isFreeCell(map, checker, footprint, sideX);
  if (!sideXFree) {
    return tl::make_unexpected(sideXFree.error());
  }

  const auto sideYFree = isFreeCell(map, checker, footprint, sideY);
  if (!sideYFree) {
    return tl::make_unexpected(sideYFree.error());
  }

  return *sideXFree && *sideYFree;
}

[[nodiscard]] auto octileHeuristic(const utils::GridCoord &from,
                                   const utils::GridCoord &targetCoord) -> double {
  const auto deltaX = std::abs(from.x - targetCoord.x);
  const auto deltaY = std::abs(from.y - targetCoord.y);
  const auto minDelta = std::min(deltaX, deltaY);
  const auto maxDelta = std::max(deltaX, deltaY);
  return static_cast<double>(maxDelta - minDelta) + (std::numbers::sqrt2 * minDelta);
}

using Node = std::pair<double, std::size_t>;
using Frontier = std::priority_queue<Node, std::vector<Node>, std::greater<>>;

auto tryRelaxNeighbor(const types::MapData &map, const ICollisionChecker &checker,
                      const types::Footprint &footprint, const StartGoalInfo &startGoal,
                      const CurrentNode &current, const Move &move, std::vector<double> &distances,
                      std::vector<std::optional<std::size_t>> &previous, Frontier &frontier)
    -> Result<void> {
  const auto neighbor =
      utils::GridCoord{.x = current.coord.x + move.dx, .y = current.coord.y + move.dy};
  if (!inBounds(map, neighbor)) {
    return {};
  }

  const auto diagonalStatus = canMoveDiagonal(map, checker, footprint, current.coord, move);
  if (!diagonalStatus) {
    return tl::make_unexpected(diagonalStatus.error());
  }
  if (!*diagonalStatus) {
    return {};
  }

  const auto neighborFree = isFreeCell(map, checker, footprint, neighbor);
  if (!neighborFree) {
    return tl::make_unexpected(neighborFree.error());
  }
  if (!*neighborFree) {
    return {};
  }

  const auto neighborIndex = utils::toIndex(map, neighbor);
  const auto nextCost = current.cost + move.cost;
  if (nextCost >= distances[neighborIndex]) {
    return {};
  }

  distances[neighborIndex] = nextCost;
  previous[neighborIndex] = current.index;
  const auto estimate = nextCost + octileHeuristic(neighbor, startGoal.goalCoord);
  frontier.emplace(estimate, neighborIndex);
  return {};
}

[[nodiscard]] auto validateInputs(const types::MapData &map, const ICollisionChecker &checker,
                                  const types::Footprint &footprint, const types::Pose &start,
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

  const auto startFree = checker.isFree(start, footprint);
  if (!startFree) {
    return tl::make_unexpected(startFree.error());
  }
  const auto goalFree = checker.isFree(goal, footprint);
  if (!goalFree) {
    return tl::make_unexpected(goalFree.error());
  }

  if (!*startFree || !*goalFree) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Start or goal cell is occupied by an obstacle."});
  }

  return StartGoalInfo{.startIndex = utils::toIndex(map, *startCell),
                       .goalIndex = utils::toIndex(map, *goalCell),
                       .startCoord = *startCell,
                       .goalCoord = *goalCell,
                       .startCenter = utils::cellCenter(map, *startCell)};
}

[[nodiscard]] auto computePrevious(const types::MapData &map, const ICollisionChecker &checker,
                                   const types::Footprint &footprint,
                                   const StartGoalInfo &startGoal)
    -> Result<std::vector<std::optional<std::size_t>>> {
  const auto totalCells = utils::cellCount(map);
  auto previous = std::vector<std::optional<std::size_t>>(totalCells, std::nullopt);
  auto distances = std::vector<double>(totalCells, std::numeric_limits<double>::infinity());

  auto frontier = Frontier{};
  distances[startGoal.startIndex] = 0.0;
  frontier.emplace(octileHeuristic(startGoal.startCoord, startGoal.goalCoord),
                   startGoal.startIndex);

  const auto sqrt2 = std::numbers::sqrt2;
  const auto moves = std::vector<Move>{
      Move{.dx = 1, .dy = 0, .cost = 1.0},    Move{.dx = -1, .dy = 0, .cost = 1.0},
      Move{.dx = 0, .dy = 1, .cost = 1.0},    Move{.dx = 0, .dy = -1, .cost = 1.0},
      Move{.dx = 1, .dy = 1, .cost = sqrt2},  Move{.dx = 1, .dy = -1, .cost = sqrt2},
      Move{.dx = -1, .dy = 1, .cost = sqrt2}, Move{.dx = -1, .dy = -1, .cost = sqrt2}};

  while (!frontier.empty()) {
    const auto [currentEstimate, current] = frontier.top();
    frontier.pop();

    const auto heuristic = octileHeuristic(utils::toCoord(map, current), startGoal.goalCoord);
    const auto currentCost = distances[current];
    if (currentEstimate > currentCost + heuristic) {
      continue;
    }

    if (current == startGoal.goalIndex) {
      break;
    }

    const auto coord = utils::toCoord(map, current);
    const auto currentNode = CurrentNode{.coord = coord, .index = current, .cost = currentCost};
    for (const auto &move : moves) {
      const auto relaxStatus = tryRelaxNeighbor(map, checker, footprint, startGoal, currentNode,
                                                move, distances, previous, frontier);
      if (!relaxStatus) {
        return tl::make_unexpected(relaxStatus.error());
      }
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

AStarPlanner::AStarPlanner(std::unique_ptr<ICollisionChecker> collisionChecker)
    : collisionChecker_{std::move(collisionChecker)} {}

auto AStarPlanner::plan(const types::MapData &map, const types::Pose &start,
                        const types::Pose &goal, const types::Footprint &footprint) const
    -> Result<types::Path> {
  const auto startGoal = validateInputs(map, *collisionChecker_, footprint, start, goal);
  if (!startGoal) {
    return tl::make_unexpected(startGoal.error());
  }

  if (startGoal->startIndex == startGoal->goalIndex) {
    return types::Path{startGoal->startCenter};
  }

  const auto previous = computePrevious(map, *collisionChecker_, footprint, *startGoal);
  if (!previous) {
    return tl::make_unexpected(previous.error());
  }

  return buildPath(map, *previous, *startGoal);
}

} // namespace ad::planning
