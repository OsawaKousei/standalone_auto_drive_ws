#include "line_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;
constexpr int kDefaultMaxExtractedLines = 40;
constexpr double kDefaultMinSegmentLength = 0.8;

struct GridVertex {
  int x;
  int y;
};

struct BoundaryEdge {
  GridVertex a;
  GridVertex b;
};

struct AxisInterval {
  int start;
  int end;
};

auto toWorldPoint(const GridVertex &vertex, double resolution) -> ad::types::Point {
  return ad::types::Point{.x = static_cast<double>(vertex.x) * resolution,
                          .y = static_cast<double>(vertex.y) * resolution};
}

auto pointDistance(const ad::types::Point &a, const ad::types::Point &b) -> double {
  return std::hypot(b.x - a.x, b.y - a.y);
}

auto buildMapLineFromSegment(const ad::types::Point &start, const ad::types::Point &end,
                             double minSegmentLength)
    -> std::optional<ad::localization::observation_model::util::MapLine> {
  const auto dx = end.x - start.x;
  const auto dy = end.y - start.y;
  const auto length = std::hypot(dx, dy);
  if (length < std::max(kEpsilon, minSegmentLength)) {
    return std::nullopt;
  }

  const auto directionX = dx / length;
  const auto directionY = dy / length;
  const auto normalX = -directionY;
  const auto normalY = directionX;
  const auto alpha = std::atan2(normalY, normalX);
  const auto rho = (normalX * start.x) + (normalY * start.y);

  const auto model = ad::localization::observation_model::util::toLineModel(
      ad::localization::observation_model::util::LineModel{.rho = rho, .alpha = alpha});
  const auto projectionStart = (directionX * start.x) + (directionY * start.y);
  const auto projectionEnd = (directionX * end.x) + (directionY * end.y);

  return ad::localization::observation_model::util::MapLine{
      .segment = ad::types::LineSegment{.start = start, .end = end},
      .model = model,
      .directionX = directionX,
      .directionY = directionY,
      .minProjection = std::min(projectionStart, projectionEnd),
      .maxProjection = std::max(projectionStart, projectionEnd)};
}

auto isOccupied(const ad::types::MapData &map, int row, int col) -> bool {
  if (row < 0 || col < 0 || row >= map.height || col >= map.width) {
    return false;
  }

  const auto width = static_cast<std::size_t>(map.width);
  const auto index = (static_cast<std::size_t>(row) * width) + static_cast<std::size_t>(col);
  return map.grid[index] > 0;
}

auto collectBoundaryEdges(const ad::types::MapData &map) -> std::vector<BoundaryEdge> {
  auto edges = std::vector<BoundaryEdge>{};

  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      if (!isOccupied(map, row, col)) {
        continue;
      }

      if (!isOccupied(map, row - 1, col)) {
        edges.push_back(BoundaryEdge{.a = GridVertex{.x = col, .y = row},
                                     .b = GridVertex{.x = col + 1, .y = row}});
      }
      if (!isOccupied(map, row, col + 1)) {
        edges.push_back(BoundaryEdge{.a = GridVertex{.x = col + 1, .y = row},
                                     .b = GridVertex{.x = col + 1, .y = row + 1}});
      }
      if (!isOccupied(map, row + 1, col)) {
        edges.push_back(BoundaryEdge{.a = GridVertex{.x = col + 1, .y = row + 1},
                                     .b = GridVertex{.x = col, .y = row + 1}});
      }
      if (!isOccupied(map, row, col - 1)) {
        edges.push_back(BoundaryEdge{.a = GridVertex{.x = col, .y = row + 1},
                                     .b = GridVertex{.x = col, .y = row}});
      }
    }
  }

  return edges;
}

auto mergeIntervals(std::vector<AxisInterval> intervals) -> std::vector<AxisInterval> {
  if (intervals.empty()) {
    return {};
  }

  std::ranges::sort(intervals, [](const auto &left, const auto &right) {
    if (left.start == right.start) {
      return left.end < right.end;
    }
    return left.start < right.start;
  });

  auto merged = std::vector<AxisInterval>{};
  merged.reserve(intervals.size());
  merged.push_back(intervals.front());

  for (std::size_t index = 1U; index < intervals.size(); ++index) {
    auto &current = merged.back();
    const auto &next = intervals[index];
    if (next.start <= current.end) {
      current.end = std::max(current.end, next.end);
      continue;
    }
    merged.push_back(next);
  }

  return merged;
}

auto mergeBoundaryEdges(const std::vector<BoundaryEdge> &edges) -> std::vector<BoundaryEdge> {
  auto horizontal = std::unordered_map<int, std::vector<AxisInterval>>{};
  auto vertical = std::unordered_map<int, std::vector<AxisInterval>>{};

  horizontal.reserve(edges.size());
  vertical.reserve(edges.size());

  for (const auto &edge : edges) {
    if (edge.a.y == edge.b.y) {
      const auto y = edge.a.y;
      const auto start = std::min(edge.a.x, edge.b.x);
      const auto end = std::max(edge.a.x, edge.b.x);
      horizontal[y].push_back(AxisInterval{.start = start, .end = end});
      continue;
    }

    if (edge.a.x == edge.b.x) {
      const auto x = edge.a.x;
      const auto start = std::min(edge.a.y, edge.b.y);
      const auto end = std::max(edge.a.y, edge.b.y);
      vertical[x].push_back(AxisInterval{.start = start, .end = end});
    }
  }

  auto mergedEdges = std::vector<BoundaryEdge>{};
  mergedEdges.reserve(edges.size());

  for (auto &[y, intervals] : horizontal) {
    for (const auto &interval : mergeIntervals(std::move(intervals))) {
      mergedEdges.push_back(BoundaryEdge{.a = GridVertex{.x = interval.start, .y = y},
                                         .b = GridVertex{.x = interval.end, .y = y}});
    }
  }

  for (auto &[x, intervals] : vertical) {
    for (const auto &interval : mergeIntervals(std::move(intervals))) {
      mergedEdges.push_back(BoundaryEdge{.a = GridVertex{.x = x, .y = interval.start},
                                         .b = GridVertex{.x = x, .y = interval.end}});
    }
  }

  return mergedEdges;
}

} // namespace

namespace ad::localization::line_extractor {

auto extractMapLinesFromMap(const types::MapData &map, const MapLineExtractionConfig &config)
    -> Result<std::vector<observation_model::util::MapLine>> {
  if (!observation_model::util::mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (map.resolution <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Map resolution must be positive."});
  }

  const auto boundaryEdges = collectBoundaryEdges(map);
  if (boundaryEdges.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Map contains no occupied contours."});
  }

  const auto mergedEdges = mergeBoundaryEdges(boundaryEdges);
  if (mergedEdges.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Failed to merge map boundary edges."});
  }

  auto sortedEdges = mergedEdges;
  std::ranges::sort(sortedEdges, [&](const auto &left, const auto &right) {
    const auto leftLength =
        pointDistance(toWorldPoint(left.a, map.resolution), toWorldPoint(left.b, map.resolution));
    const auto rightLength =
        pointDistance(toWorldPoint(right.a, map.resolution), toWorldPoint(right.b, map.resolution));
    return leftLength > rightLength;
  });

  auto lines = std::vector<observation_model::util::MapLine>{};
  const auto maxExtractedLines = std::max(1, config.maxLines);
  const auto minSegmentLength = std::max(kEpsilon, config.minSegmentLength);

  lines.reserve(static_cast<std::size_t>(maxExtractedLines));
  for (const auto &edge : sortedEdges) {
    if (static_cast<int>(lines.size()) >= maxExtractedLines) {
      break;
    }

    const auto start = toWorldPoint(edge.a, map.resolution);
    const auto end = toWorldPoint(edge.b, map.resolution);
    if (pointDistance(start, end) < minSegmentLength) {
      continue;
    }

    const auto candidate = buildMapLineFromSegment(start, end, minSegmentLength);
    if (!candidate) {
      continue;
    }

    lines.push_back(*candidate);
  }

  if (lines.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No line segments extracted from map contours."});
  }

  return lines;
}

auto extractMapLinesFromMap(const types::MapData &map)
    -> Result<std::vector<observation_model::util::MapLine>> {
  return extractMapLinesFromMap(
      map, MapLineExtractionConfig{.maxLines = kDefaultMaxExtractedLines,
                                   .minSegmentLength = kDefaultMinSegmentLength});
}

} // namespace ad::localization::line_extractor
