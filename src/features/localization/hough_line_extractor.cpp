#include "hough_line_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;

struct HoughCandidate {
  double rho;
  double alpha;
  int votes;
};

struct HoughParams {
  double thetaMin;
  double thetaStep;
  double rhoMin;
  double rhoStep;
  int thetaBins;
  int rhoBins;
  std::size_t thetaBinsSize;
  std::size_t rhoBinsSize;
};

auto buildHoughParams(const ad::localization::HoughConfig &config, double maxRho) -> HoughParams {
  const auto thetaMin = -0.5 * std::numbers::pi;
  const auto thetaMax = 0.5 * std::numbers::pi;
  const auto thetaStep = (thetaMax - thetaMin) / static_cast<double>(config.thetaBins - 1);
  const auto rhoMin = -maxRho;
  const auto rhoMax = maxRho;
  const auto rhoStep = (rhoMax - rhoMin) / static_cast<double>(config.rhoBins - 1);

  return HoughParams{.thetaMin = thetaMin,
                     .thetaStep = thetaStep,
                     .rhoMin = rhoMin,
                     .rhoStep = rhoStep,
                     .thetaBins = config.thetaBins,
                     .rhoBins = config.rhoBins,
                     .thetaBinsSize = static_cast<std::size_t>(config.thetaBins),
                     .rhoBinsSize = static_cast<std::size_t>(config.rhoBins)};
}

auto buildAccumulator(const std::vector<ad::types::Point> &points, const HoughParams &params)
    -> std::vector<int> {
  auto accumulator = std::vector<int>(params.thetaBinsSize * params.rhoBinsSize, 0);

  for (const auto &point : points) {
    for (int thetaIndex = 0; thetaIndex < params.thetaBins; ++thetaIndex) {
      const auto theta = params.thetaMin + (params.thetaStep * static_cast<double>(thetaIndex));
      const auto rho = (point.x * std::cos(theta)) + (point.y * std::sin(theta));
      const auto rhoIndex = static_cast<int>(std::lround((rho - params.rhoMin) / params.rhoStep));
      if (rhoIndex < 0 || rhoIndex >= params.rhoBins) {
        continue;
      }
      const auto index = (static_cast<std::size_t>(thetaIndex) * params.rhoBinsSize) +
                         static_cast<std::size_t>(rhoIndex);
      ++accumulator[index];
    }
  }

  return accumulator;
}

auto collectCandidates(const std::vector<int> &accumulator, const HoughParams &params,
                       const ad::localization::HoughConfig &config) -> std::vector<HoughCandidate> {
  auto candidates = std::vector<HoughCandidate>{};
  for (int thetaIndex = 0; thetaIndex < params.thetaBins; ++thetaIndex) {
    for (int rhoIndex = 0; rhoIndex < params.rhoBins; ++rhoIndex) {
      const auto index = (static_cast<std::size_t>(thetaIndex) * params.rhoBinsSize) +
                         static_cast<std::size_t>(rhoIndex);
      const auto votes = accumulator[index];
      if (votes < config.minVotes) {
        continue;
      }
      const auto theta = params.thetaMin + (params.thetaStep * static_cast<double>(thetaIndex));
      const auto rho = params.rhoMin + (params.rhoStep * static_cast<double>(rhoIndex));
      candidates.push_back({rho, theta, votes});
    }
  }

  return candidates;
}

auto isTooCloseToExisting(const ad::localization::util::LineModel &normalized,
                          const std::vector<ad::localization::util::MapLine> &lines,
                          const ad::localization::HoughConfig &config) -> bool {
  return std::ranges::any_of(lines, [&](const auto &existing) {
    const auto rhoDiff = std::abs(existing.model.rho - normalized.rho);
    const auto alphaDiff =
        std::abs(ad::localization::util::normalizeAngle(existing.model.alpha - normalized.alpha));
    return rhoDiff <= config.mergeRho && alphaDiff <= config.mergeTheta;
  });
}

auto buildLineFromCandidate(const HoughCandidate &candidate,
                            const std::vector<ad::types::Point> &points,
                            const ad::localization::HoughConfig &config,
                            const ad::localization::util::LineModel &normalized)
    -> std::optional<ad::localization::util::MapLine> {
  const auto normalX = std::cos(candidate.alpha);
  const auto normalY = std::sin(candidate.alpha);
  const auto tangentX = -normalY;
  const auto tangentY = normalX;

  auto minProjection = std::optional<double>{};
  auto maxProjection = std::optional<double>{};
  for (const auto &point : points) {
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - candidate.rho);
    if (distance > config.inlierDistance) {
      continue;
    }

    const auto projection = (tangentX * point.x) + (tangentY * point.y);
    if (!minProjection || projection < *minProjection) {
      minProjection = projection;
    }
    if (!maxProjection || projection > *maxProjection) {
      maxProjection = projection;
    }
  }

  if (!minProjection || !maxProjection) {
    return std::nullopt;
  }

  const auto segmentLength = std::abs(*maxProjection - *minProjection);
  if (segmentLength < config.minSegmentLength) {
    return std::nullopt;
  }

  const auto startPoint =
      ad::types::Point{.x = (tangentX * (*minProjection)) + (normalX * candidate.rho),
                       .y = (tangentY * (*minProjection)) + (normalY * candidate.rho)};
  const auto endPoint =
      ad::types::Point{.x = (tangentX * (*maxProjection)) + (normalX * candidate.rho),
                       .y = (tangentY * (*maxProjection)) + (normalY * candidate.rho)};

  const auto segmentDeltaX = endPoint.x - startPoint.x;
  const auto segmentDeltaY = endPoint.y - startPoint.y;
  const auto segmentDistance = std::hypot(segmentDeltaX, segmentDeltaY);
  if (segmentDistance < kEpsilon) {
    return std::nullopt;
  }

  const auto directionX = segmentDeltaX / segmentDistance;
  const auto directionY = segmentDeltaY / segmentDistance;
  auto minProjValue = (directionX * startPoint.x) + (directionY * startPoint.y);
  auto maxProjValue = (directionX * endPoint.x) + (directionY * endPoint.y);
  if (minProjValue > maxProjValue) {
    std::swap(minProjValue, maxProjValue);
  }

  return ad::localization::util::MapLine{ad::types::LineSegment{startPoint, endPoint},
                                         normalized,
                                         directionX,
                                         directionY,
                                         minProjValue,
                                         maxProjValue};
}

} // namespace

namespace ad::localization::hough {

auto extractMapLinesFromMap(const types::MapData &map, const HoughConfig &config)
    -> Result<std::vector<util::MapLine>> {
  if (!util::mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (config.thetaBins < 2 || config.rhoBins < 2 || config.minVotes <= 0 || config.maxLines <= 0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Hough configuration is invalid."});
  }

  const auto points = util::collectOccupiedPoints(map);
  if (points.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Map contains no occupied cells."});
  }

  const auto maxRho = std::hypot(map.width * map.resolution, map.height * map.resolution);
  if (maxRho <= kEpsilon) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map resolution too small for Hough transform."});
  }

  const auto params = buildHoughParams(config, maxRho);
  const auto accumulator = buildAccumulator(points, params);
  auto candidates = collectCandidates(accumulator, params, config);

  if (candidates.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No Hough candidates met the vote threshold."});
  }

  std::ranges::sort(candidates, [](const auto &left, const auto &right) -> bool {
    return left.votes > right.votes;
  });

  auto lines = std::vector<util::MapLine>{};
  for (const auto &candidate : candidates) {
    if (static_cast<int>(lines.size()) >= config.maxLines) {
      break;
    }

    const auto normalized =
        util::toLineModel(util::LineModel{.rho = candidate.rho, .alpha = candidate.alpha});
    if (isTooCloseToExisting(normalized, lines, config)) {
      continue;
    }

    const auto line = buildLineFromCandidate(candidate, points, config, normalized);
    if (!line) {
      continue;
    }
    lines.push_back(*line);
  }

  if (lines.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No line segments extracted from Hough candidates."});
  }

  return lines;
}

} // namespace ad::localization::hough
