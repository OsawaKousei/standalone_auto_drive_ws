#include "scan_line_extractor.hpp"

#include "line_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace {

constexpr double kLineExtractionDistanceMin = 1e-6;
constexpr double kLineExtractionSpanMin = 1e-3;

auto isTooCloseToExisting(const ad::localization::util::LineModel &candidate,
                          const std::vector<ad::localization::util::MapLine> &lines,
                          const ad::localization::RansacLineExtractionConfig &lineExtraction)
    -> bool {
  return std::any_of(lines.begin(), lines.end(), [&](const auto &existing) {
    const auto rhoDiff = std::abs(existing.model.rho - candidate.rho);
    const auto alphaDiff =
        std::abs(ad::localization::util::normalizeAngle(existing.model.alpha - candidate.alpha));
    return rhoDiff <= lineExtraction.mergeRho && alphaDiff <= lineExtraction.mergeTheta;
  });
}

auto buildMapLineFromInliers(const ad::localization::util::LineModel &model,
                             const std::vector<ad::types::Point> &points,
                             const std::vector<std::size_t> &inlierIndices)
    -> std::optional<ad::localization::util::MapLine> {
  if (inlierIndices.size() < 2U) {
    return std::nullopt;
  }

  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);
  const auto tangentX = -normalY;
  const auto tangentY = normalX;

  auto minProjection = std::numeric_limits<double>::infinity();
  auto maxProjection = -std::numeric_limits<double>::infinity();
  for (const auto index : inlierIndices) {
    const auto &point = points[index];
    const auto projection = (tangentX * point.x) + (tangentY * point.y);
    minProjection = std::min(minProjection, projection);
    maxProjection = std::max(maxProjection, projection);
  }

  if (!std::isfinite(minProjection) || !std::isfinite(maxProjection) ||
      (maxProjection - minProjection) < kLineExtractionSpanMin) {
    return std::nullopt;
  }

  return ad::localization::line_geometry::buildMapLineFromModelAndProjectionSpan(
      model, minProjection, maxProjection, kLineExtractionSpanMin);
}

} // namespace

namespace ad::localization::scan_line_extractor {

auto collectScanPoints(const types::LidarScan &scan) -> std::vector<types::Point> {
  auto points = std::vector<types::Point>{};
  points.reserve(scan.ranges.size());

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }

    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    points.push_back(types::Point{.x = range * std::cos(angle), .y = range * std::sin(angle)});
  }

  return points;
}

auto extractLinesFromPointsRansac(const std::vector<types::Point> &points,
                                  const RansacScanLineExtractionConfig &config)
    -> Result<std::vector<util::MapLine>> {
  if (points.size() < 2U) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Point set is too small for RANSAC."});
  }

  const auto maxLines = std::max(1, config.lineExtraction.maxLines);
  const auto inlierDistance =
      std::max(kLineExtractionDistanceMin, config.lineExtraction.inlierDistance);
  const auto minInlierSpan =
      std::max(kLineExtractionSpanMin, config.lineExtraction.minSegmentLength);

  auto remainingPoints = points;
  auto lines = std::vector<util::MapLine>{};
  lines.reserve(static_cast<std::size_t>(maxLines));

  for (int lineIndex = 0; lineIndex < maxLines && remainingPoints.size() >= 2U; ++lineIndex) {
    const auto ratioDrivenMinInliers =
        static_cast<std::size_t>(std::ceil(0.02 * static_cast<double>(remainingPoints.size())));
    const auto requestedMinInliers = std::min<std::size_t>(
        config.ransac.minInliers, std::max<std::size_t>(2U, ratioDrivenMinInliers));
    const auto minInliers = std::max<std::size_t>(
        2U, std::min<std::size_t>(requestedMinInliers, remainingPoints.size()));
    const auto minInlierRatio = std::max(0.002, std::min(config.ransac.minInlierRatio, 0.08));

    const auto fit = ransac::fitLineToPoints(
        remainingPoints, ransac::PointLineRansacConfig{.maxIterations = config.ransac.maxIterations,
                                                       .inlierDistance = inlierDistance,
                                                       .minInliers = minInliers,
                                                       .minInlierRatio = minInlierRatio,
                                                       .minInlierSpan = minInlierSpan});
    if (!fit) {
      break;
    }

    const auto line = buildMapLineFromInliers(fit->fit.model, remainingPoints, fit->inlierIndices);
    if (line && !isTooCloseToExisting(fit->fit.model, lines, config.lineExtraction)) {
      lines.push_back(*line);
    }

    auto inlierMask = std::vector<bool>(remainingPoints.size(), false);
    for (const auto index : fit->inlierIndices) {
      inlierMask[index] = true;
    }

    auto nextPoints = std::vector<types::Point>{};
    nextPoints.reserve(remainingPoints.size() - fit->inlierIndices.size());
    for (std::size_t index = 0; index < remainingPoints.size(); ++index) {
      if (!inlierMask[index]) {
        nextPoints.push_back(remainingPoints[index]);
      }
    }
    remainingPoints = std::move(nextPoints);
  }

  if (lines.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No line segments extracted by RANSAC."});
  }

  return lines;
}

} // namespace ad::localization::scan_line_extractor
