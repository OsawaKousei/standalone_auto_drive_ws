#include "line_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;

struct GridVertex {
  int x;
  int y;
};

struct BoundaryEdge {
  GridVertex a;
  GridVertex b;
};

struct PointHash {
  auto operator()(const GridVertex &vertex) const noexcept -> std::size_t {
    const auto ux = static_cast<std::uint32_t>(vertex.x);
    const auto uy = static_cast<std::uint32_t>(vertex.y);
    return (static_cast<std::size_t>(ux) << 32U) ^ static_cast<std::size_t>(uy);
  }
};

auto operator==(const GridVertex &lhs, const GridVertex &rhs) -> bool {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

auto toWorldPoint(const GridVertex &vertex, double resolution) -> ad::types::Point {
  return ad::types::Point{.x = static_cast<double>(vertex.x) * resolution,
                          .y = static_cast<double>(vertex.y) * resolution};
}

auto pointDistance(const ad::types::Point &a, const ad::types::Point &b) -> double {
  return std::hypot(b.x - a.x, b.y - a.y);
}

auto perpendicularDistance(const ad::types::Point &point, const ad::types::Point &lineStart,
                           const ad::types::Point &lineEnd) -> double {
  const auto dx = lineEnd.x - lineStart.x;
  const auto dy = lineEnd.y - lineStart.y;
  const auto norm = std::hypot(dx, dy);
  if (norm < kEpsilon) {
    return pointDistance(point, lineStart);
  }

  const auto cross = std::abs((dx * (lineStart.y - point.y)) - ((lineStart.x - point.x) * dy));
  return cross / norm;
}

auto douglasPeuckerRecursive(const std::vector<ad::types::Point> &points, std::size_t startIndex,
                             std::size_t endIndex, double epsilon, std::vector<bool> &keepMask)
    -> void {
  if (endIndex <= startIndex + 1U) {
    return;
  }

  auto maxDistance = 0.0;
  auto splitIndex = startIndex;
  for (auto index = startIndex + 1U; index < endIndex; ++index) {
    const auto distance =
        perpendicularDistance(points[index], points[startIndex], points[endIndex]);
    if (distance > maxDistance) {
      maxDistance = distance;
      splitIndex = index;
    }
  }

  if (maxDistance <= epsilon) {
    return;
  }

  keepMask[splitIndex] = true;
  douglasPeuckerRecursive(points, startIndex, splitIndex, epsilon, keepMask);
  douglasPeuckerRecursive(points, splitIndex, endIndex, epsilon, keepMask);
}

auto simplifyPolylineDouglasPeucker(const std::vector<ad::types::Point> &points, double epsilon)
    -> std::vector<ad::types::Point> {
  if (points.size() <= 2U) {
    return points;
  }

  auto keepMask = std::vector<bool>(points.size(), false);
  keepMask.front() = true;
  keepMask.back() = true;
  douglasPeuckerRecursive(points, 0U, points.size() - 1U, epsilon, keepMask);

  auto simplified = std::vector<ad::types::Point>{};
  simplified.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (keepMask[index]) {
      simplified.push_back(points[index]);
    }
  }

  return simplified;
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

auto traceContours(const std::vector<BoundaryEdge> &edges) -> std::vector<std::vector<GridVertex>> {
  auto adjacency = std::unordered_map<GridVertex, std::vector<std::size_t>, PointHash>{};
  adjacency.reserve(edges.size() * 2U);
  for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
    adjacency[edges[edgeIndex].a].push_back(edgeIndex);
    adjacency[edges[edgeIndex].b].push_back(edgeIndex);
  }

  auto used = std::vector<bool>(edges.size(), false);
  auto contours = std::vector<std::vector<GridVertex>>{};

  for (std::size_t seedIndex = 0; seedIndex < edges.size(); ++seedIndex) {
    if (used[seedIndex]) {
      continue;
    }

    auto contour = std::vector<GridVertex>{};
    auto currentEdgeIndex = seedIndex;
    auto currentVertex = edges[currentEdgeIndex].a;

    contour.push_back(currentVertex);

    while (true) {
      used[currentEdgeIndex] = true;
      const auto &edge = edges[currentEdgeIndex];
      const auto nextVertex = (edge.a == currentVertex) ? edge.b : edge.a;
      contour.push_back(nextVertex);

      const auto it = adjacency.find(nextVertex);
      if (it == adjacency.end()) {
        break;
      }

      auto nextEdgeIndex = std::optional<std::size_t>{};
      for (const auto candidateEdgeIndex : it->second) {
        if (!used[candidateEdgeIndex]) {
          nextEdgeIndex = candidateEdgeIndex;
          break;
        }
      }

      if (!nextEdgeIndex) {
        break;
      }

      currentVertex = nextVertex;
      currentEdgeIndex = *nextEdgeIndex;
    }

    if (contour.size() >= 3U) {
      contours.push_back(std::move(contour));
    }
  }

  return contours;
}

auto simplifyContour(const std::vector<GridVertex> &contour, double resolution, double epsilon)
    -> std::vector<ad::types::Point> {
  auto points = std::vector<ad::types::Point>{};
  points.reserve(contour.size() + 1U);
  for (const auto &vertex : contour) {
    points.push_back(toWorldPoint(vertex, resolution));
  }

  if (points.front().x != points.back().x || points.front().y != points.back().y) {
    points.push_back(points.front());
  }

  auto simplified = simplifyPolylineDouglasPeucker(points, epsilon);
  if (simplified.size() >= 2U && (simplified.front().x != simplified.back().x ||
                                  simplified.front().y != simplified.back().y)) {
    simplified.push_back(simplified.front());
  }

  return simplified;
}

auto buildMapLine(const ad::types::Point &start, const ad::types::Point &end)
    -> std::optional<ad::localization::util::MapLine> {
  const auto dx = end.x - start.x;
  const auto dy = end.y - start.y;
  const auto length = std::hypot(dx, dy);
  if (length < kEpsilon) {
    return std::nullopt;
  }

  const auto directionX = dx / length;
  const auto directionY = dy / length;
  const auto alphaRaw = std::atan2(dy, dx) + (0.5 * std::numbers::pi);
  const auto normalX = std::cos(alphaRaw);
  const auto normalY = std::sin(alphaRaw);
  const auto rhoRaw = (normalX * start.x) + (normalY * start.y);
  const auto model = ad::localization::util::toLineModel(
      ad::localization::util::LineModel{.rho = rhoRaw, .alpha = alphaRaw});

  auto minProjection = (directionX * start.x) + (directionY * start.y);
  auto maxProjection = (directionX * end.x) + (directionY * end.y);
  if (minProjection > maxProjection) {
    std::swap(minProjection, maxProjection);
  }

  return ad::localization::util::MapLine{ad::types::LineSegment{.start = start, .end = end},
                                         model,
                                         directionX,
                                         directionY,
                                         minProjection,
                                         maxProjection};
}

auto isTooCloseToExisting(const ad::localization::util::LineModel &candidate,
                          const std::vector<ad::localization::util::MapLine> &lines,
                          const ad::localization::HoughConfig &config) -> bool {
  return std::ranges::any_of(lines, [&](const auto &existing) {
    const auto rhoDiff = std::abs(existing.model.rho - candidate.rho);
    const auto alphaDiff =
        std::abs(ad::localization::util::normalizeAngle(existing.model.alpha - candidate.alpha));
    return rhoDiff <= config.mergeRho && alphaDiff <= config.mergeTheta;
  });
}

} // namespace

namespace ad::localization::line_extractor {

auto extractMapLinesFromMap(const types::MapData &map, const HoughConfig &config)
    -> Result<std::vector<util::MapLine>> {
  if (!util::mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (config.maxLines <= 0 || config.minSegmentLength <= 0.0) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Line extractor configuration is invalid."});
  }

  const auto boundaryEdges = collectBoundaryEdges(map);
  if (boundaryEdges.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Map contains no occupied contours."});
  }

  const auto contours = traceContours(boundaryEdges);
  if (contours.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Failed to construct map contours."});
  }

  const auto simplificationEpsilon = std::max(map.resolution * 0.5, config.inlierDistance);

  auto lines = std::vector<util::MapLine>{};
  lines.reserve(static_cast<std::size_t>(config.maxLines));

  for (const auto &contour : contours) {
    if (static_cast<int>(lines.size()) >= config.maxLines) {
      break;
    }

    const auto simplified = simplifyContour(contour, map.resolution, simplificationEpsilon);
    if (simplified.size() < 2U) {
      continue;
    }

    for (std::size_t index = 1U; index < simplified.size(); ++index) {
      if (static_cast<int>(lines.size()) >= config.maxLines) {
        break;
      }

      const auto &start = simplified[index - 1U];
      const auto &end = simplified[index];
      if (pointDistance(start, end) < config.minSegmentLength) {
        continue;
      }

      const auto line = buildMapLine(start, end);
      if (!line) {
        continue;
      }
      if (isTooCloseToExisting(line->model, lines, config)) {
        continue;
      }

      lines.push_back(*line);
    }
  }

  if (lines.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No line segments extracted from map contours."});
  }

  return lines;
}

} // namespace ad::localization::line_extractor
