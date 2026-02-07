#include "collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <span>
#include <vector>

namespace ad::simulation {

namespace {

[[nodiscard]] auto mapHasConsistentGrid(const types::MapData &map) -> bool {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.grid.size() == expectedCells;
}

[[nodiscard]] auto validateMap(const types::MapData &map) -> Result<void> {
  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Map dimensions must be positive."});
  }

  if (!mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  return {};
}

[[nodiscard]] auto validateFootprint(const types::Footprint &footprint) -> Result<void> {
  if (footprint.vertices.empty()) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Footprint has no vertices."});
  }
  return {};
}

[[nodiscard]] auto transformFootprint(const types::Footprint &footprint, const types::Pose &pose)
    -> std::vector<types::Point> {
  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);

  auto vertices = std::vector<types::Point>{};
  vertices.reserve(footprint.vertices.size());
  for (const auto &vertex : footprint.vertices) {
    const auto globalX = pose.x + (vertex.x * cosTheta) - (vertex.y * sinTheta);
    const auto globalY = pose.y + (vertex.x * sinTheta) + (vertex.y * cosTheta);
    vertices.push_back(types::Point{globalX, globalY});
  }
  return vertices;
}

[[nodiscard]] auto pointInPolygon(std::span<const types::Point> polygon, const types::Point &point)
    -> bool {
  if (polygon.size() < 3U) {
    return false;
  }

  auto inside = false;
  const auto lastIndex = polygon.size();
  for (const auto index : std::views::iota(std::size_t{0}, lastIndex)) {
    const auto &current = polygon[index];
    const auto &next = polygon[(index + 1U) % lastIndex];
    if (current.y == next.y) {
      continue;
    }
    const auto crosses =
        ((current.y > point.y) != (next.y > point.y)) &&
        (point.x < (next.x - current.x) * (point.y - current.y) / (next.y - current.y) + current.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

[[nodiscard]] auto clampCellIndex(int value, int limit) -> int {
  return std::clamp(value, 0, std::max(limit - 1, 0));
}

[[nodiscard]] auto cellCenter(const types::MapData &map, int cellX, int cellY) -> types::Point {
  const auto half = 0.5 * map.resolution;
  return types::Point{(static_cast<double>(cellX) * map.resolution) + half,
                      (static_cast<double>(cellY) * map.resolution) + half};
}

[[nodiscard]] auto isCellOccupied(const types::MapData &map, int cellX, int cellY) -> bool {
  const auto width = static_cast<std::size_t>(map.width);
  const auto index = (static_cast<std::size_t>(cellY) * width) + static_cast<std::size_t>(cellX);
  return map.grid[index] > 0;
}

[[nodiscard]] auto footprintCollides(const types::MapData &map,
                                     std::span<const types::Point> footprint) -> bool {
  auto minX = footprint.front().x;
  auto maxX = footprint.front().x;
  auto minY = footprint.front().y;
  auto maxY = footprint.front().y;
  for (const auto &vertex : footprint) {
    minX = std::min(minX, vertex.x);
    maxX = std::max(maxX, vertex.x);
    minY = std::min(minY, vertex.y);
    maxY = std::max(maxY, vertex.y);
  }

  const auto minCellX =
      clampCellIndex(static_cast<int>(std::floor(minX / map.resolution)), map.width);
  const auto maxCellX =
      clampCellIndex(static_cast<int>(std::floor(maxX / map.resolution)), map.width);
  const auto minCellY =
      clampCellIndex(static_cast<int>(std::floor(minY / map.resolution)), map.height);
  const auto maxCellY =
      clampCellIndex(static_cast<int>(std::floor(maxY / map.resolution)), map.height);

  const auto xIndices = std::views::iota(minCellX, maxCellX + 1);
  const auto yIndices = std::views::iota(minCellY, maxCellY + 1);
  for (const auto y : yIndices) {
    for (const auto x : xIndices) {
      if (!isCellOccupied(map, x, y)) {
        continue;
      }
      const auto center = cellCenter(map, x, y);
      if (pointInPolygon(footprint, center)) {
        return true;
      }
    }
  }

  return false;
}

[[nodiscard]] auto interpolatePose(const types::Pose &start, const types::Pose &end, double t)
    -> types::Pose {
  return types::Pose{start.x + (end.x - start.x) * t, start.y + (end.y - start.y) * t,
                     start.theta + (end.theta - start.theta) * t};
}

} // namespace

CollisionChecker::CollisionChecker(const types::MapData &map, const types::Footprint &footprint,
                                   CollisionCheckConfig config)
    : map_{map}, footprint_{footprint}, config_{config} {}

auto CollisionChecker::isPoseCollisionFree(const types::Pose &pose) const -> Result<bool> {
  const auto mapStatus = validateMap(map_);
  if (!mapStatus) {
    return tl::make_unexpected(mapStatus.error());
  }

  const auto footprintStatus = validateFootprint(footprint_);
  if (!footprintStatus) {
    return tl::make_unexpected(footprintStatus.error());
  }

  const auto footprintWorld = transformFootprint(footprint_, pose);
  const auto collides = footprintCollides(map_, footprintWorld);
  return !collides;
}

auto CollisionChecker::checkTrajectory(const types::Pose &start, const types::Pose &end) const
    -> Result<bool> {
  const auto mapStatus = validateMap(map_);
  if (!mapStatus) {
    return tl::make_unexpected(mapStatus.error());
  }

  const auto footprintStatus = validateFootprint(footprint_);
  if (!footprintStatus) {
    return tl::make_unexpected(footprintStatus.error());
  }

  if (config_.maxTranslationStep <= 0.0 || config_.maxRotationStep <= 0.0) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Collision step sizes must be positive."});
  }

  const auto deltaX = end.x - start.x;
  const auto deltaY = end.y - start.y;
  const auto deltaTheta = end.theta - start.theta;
  const auto translation = std::hypot(deltaX, deltaY);
  const auto translationSteps =
      static_cast<int>(std::ceil(translation / config_.maxTranslationStep));
  const auto rotationSteps =
      static_cast<int>(std::ceil(std::abs(deltaTheta) / config_.maxRotationStep));
  const auto stepCount = std::max({1, translationSteps, rotationSteps});

  const auto stepIndices = std::views::iota(1, stepCount + 1);
  const auto isFree = std::ranges::all_of(stepIndices, [&](int stepIndex) {
    const auto t = static_cast<double>(stepIndex) / static_cast<double>(stepCount);
    const auto pose = interpolatePose(start, end, t);
    const auto footprintWorld = transformFootprint(footprint_, pose);
    return !footprintCollides(map_, footprintWorld);
  });

  return isFree;
}

} // namespace ad::simulation
