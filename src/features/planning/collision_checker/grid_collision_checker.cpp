#include "grid_collision_checker.hpp"
#include "../planner_utils.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <vector>

namespace ad::planning {

namespace {

[[nodiscard]] auto inBounds(const types::MapData &map, const utils::GridCoord &coord) -> bool {
  return coord.x >= 0 && coord.y >= 0 && coord.x < map.width && coord.y < map.height;
}

[[nodiscard]] auto computeFootprintRadius(const types::Footprint &footprint) -> Result<double> {
  if (footprint.vertices.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Footprint has no vertices."});
  }

  auto maxSquared = 0.0;
  for (const auto &vertex : footprint.vertices) {
    const auto squared = (vertex.x * vertex.x) + (vertex.y * vertex.y);
    maxSquared = std::max(maxSquared, squared);
  }
  return std::sqrt(maxSquared);
}

[[nodiscard]] auto buildOffsets(int radiusCells) -> std::vector<utils::GridOffset> {
  if (radiusCells <= 0) {
    return {};
  }

  auto offsets = std::vector<utils::GridOffset>{};
  const auto radiusCellsSize = static_cast<std::size_t>(radiusCells);
  const auto diameter = (radiusCellsSize * 2U) + 1U;
  offsets.reserve(diameter * diameter);

  const auto radiusSquared = radiusCells * radiusCells;
  for (const auto deltaX : std::views::iota(-radiusCells, radiusCells + 1)) {
    for (const auto deltaY : std::views::iota(-radiusCells, radiusCells + 1)) {
      const auto distanceSquared = (deltaX * deltaX) + (deltaY * deltaY);
      if (distanceSquared <= radiusSquared) {
        offsets.push_back(utils::GridOffset{.dx = deltaX, .dy = deltaY});
      }
    }
  }
  return offsets;
}

[[nodiscard]] auto inflateGrid(const types::MapData &map, int radiusCells)
    -> std::vector<std::int8_t> {
  if (radiusCells <= 0) {
    return map.grid;
  }

  const auto offsets = buildOffsets(radiusCells);
  auto inflated = map.grid;

  for (const auto yValue : std::views::iota(0, map.height)) {
    for (const auto xValue : std::views::iota(0, map.width)) {
      const auto coord = utils::GridCoord{.x = xValue, .y = yValue};
      if (!utils::isObstacle(map, coord)) {
        continue;
      }

      for (const auto &offset : offsets) {
        const auto target = utils::GridCoord{.x = coord.x + offset.dx, .y = coord.y + offset.dy};
        if (!inBounds(map, target)) {
          continue;
        }
        const auto index = utils::toIndex(map, target);
        inflated[index] = 1;
      }
    }
  }
  return inflated;
}

} // namespace

GridCollisionChecker::GridCollisionChecker(types::MapData inflatedMap, double footprintRadius)
    : inflatedMap_{std::move(inflatedMap)}, footprintRadius_{footprintRadius} {}

auto GridCollisionChecker::create(const types::MapData &map, const types::Footprint &footprint)
    -> Result<std::unique_ptr<ICollisionChecker>> {
  const auto mapStatus = utils::isValidMap(map);
  if (!mapStatus) {
    return tl::make_unexpected(mapStatus.error());
  }

  const auto radius = computeFootprintRadius(footprint);
  if (!radius) {
    return tl::make_unexpected(radius.error());
  }

  const auto radiusCells = static_cast<int>(std::ceil(*radius / map.resolution));
  auto inflatedGrid = inflateGrid(map, radiusCells);

  auto inflatedMap = types::MapData{.width = map.width,
                                    .height = map.height,
                                    .resolution = map.resolution,
                                    .grid = std::move(inflatedGrid)};
  return Result<std::unique_ptr<ICollisionChecker>>{
      tl::in_place, std::make_unique<GridCollisionChecker>(std::move(inflatedMap), *radius)};
}

auto GridCollisionChecker::isFree(const types::Pose &pose, const types::Footprint &footprint) const
    -> Result<bool> {
  const auto radius = computeFootprintRadius(footprint);
  if (!radius) {
    return tl::make_unexpected(radius.error());
  }

  constexpr double kTolerance = 1e-6;
  if (std::abs(*radius - footprintRadius_) > kTolerance) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Footprint radius does not match checker configuration."});
  }

  const auto cell = utils::worldToCell(inflatedMap_, pose);
  if (!cell) {
    return tl::make_unexpected(cell.error());
  }

  return !utils::isObstacle(inflatedMap_, *cell);
}

} // namespace ad::planning
