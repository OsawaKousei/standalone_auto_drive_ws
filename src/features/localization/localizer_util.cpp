#include "localizer_util.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <ranges>
#include <vector>

namespace ad::localization::util {

namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kGateEpsilon = 1e-12;
constexpr double kTwo = 2.0;
constexpr double kTwoPi = kTwo * std::numbers::pi;

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

auto buildHoughParams(const HoughConfig &config, double maxRho) -> HoughParams {
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

auto buildAccumulator(const std::vector<types::Point> &points, const HoughParams &params)
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
                       const HoughConfig &config) -> std::vector<HoughCandidate> {
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

auto isTooCloseToExisting(const LineModel &normalized, const std::vector<MapLine> &lines,
                          const HoughConfig &config) -> bool {
  return std::ranges::any_of(lines, [&](const auto &existing) {
    const auto rhoDiff = std::abs(existing.model.rho - normalized.rho);
    const auto alphaDiff = std::abs(normalizeAngle(existing.model.alpha - normalized.alpha));
    return rhoDiff <= config.mergeRho && alphaDiff <= config.mergeTheta;
  });
}

auto buildLineFromCandidate(const HoughCandidate &candidate,
                            const std::vector<types::Point> &points, const HoughConfig &config,
                            const LineModel &normalized) -> std::optional<MapLine> {
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
      types::Point{.x = (tangentX * (*minProjection)) + (normalX * candidate.rho),
                   .y = (tangentY * (*minProjection)) + (normalY * candidate.rho)};
  const auto endPoint =
      types::Point{.x = (tangentX * (*maxProjection)) + (normalX * candidate.rho),
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

  return MapLine{types::LineSegment{startPoint, endPoint},
                 normalized,
                 directionX,
                 directionY,
                 minProjValue,
                 maxProjValue};
}

} // namespace

auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - std::numbers::pi;
}

auto mapHasConsistentGrid(const types::MapData &map) -> bool {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.grid.size() == expectedCells;
}

auto collectOccupiedPoints(const types::MapData &map) -> std::vector<types::Point> {
  const auto width = static_cast<std::size_t>(map.width);
  const auto height = static_cast<std::size_t>(map.height);
  auto points = std::vector<types::Point>{};

  for (std::size_t rowIndex = 0; rowIndex < height; ++rowIndex) {
    for (std::size_t colIndex = 0; colIndex < width; ++colIndex) {
      const auto index = (rowIndex * width) + colIndex;
      if (map.grid[index] <= 0) {
        continue;
      }

      const auto xValue = (static_cast<double>(colIndex) + 0.5) * map.resolution;
      const auto yValue = (static_cast<double>(rowIndex) + 0.5) * map.resolution;
      points.push_back(types::Point{.x = xValue, .y = yValue});
    }
  }

  return points;
}

auto passesGate(const GateCheck &check) -> bool {
  if (check.variance <= 0.0) {
    return false;
  }
  const auto normalized = (check.residual * check.residual) / check.variance;
  return normalized <= check.threshold;
}

auto toLineModel(LineModel raw) -> LineModel {
  auto normalizedRho = raw.rho;
  auto normalizedAlpha = normalizeAngle(raw.alpha);
  if (normalizedRho < 0.0) {
    normalizedRho = -normalizedRho;
    normalizedAlpha = normalizeAngle(normalizedAlpha + std::numbers::pi);
  }
  return LineModel{.rho = normalizedRho, .alpha = normalizedAlpha};
}

auto fitLine(const std::vector<types::Point> &points) -> std::optional<LineFit> {
  if (points.size() < 2U) {
    return std::nullopt;
  }

  double meanX = 0.0;
  double meanY = 0.0;
  for (const auto &point : points) {
    meanX += point.x;
    meanY += point.y;
  }
  const auto count = static_cast<double>(points.size());
  meanX /= count;
  meanY /= count;

  auto momentSums = std::array<double, 3>{0.0, 0.0, 0.0};
  for (const auto &point : points) {
    const auto deltaX = point.x - meanX;
    const auto deltaY = point.y - meanY;
    momentSums[0] += deltaX * deltaX;
    momentSums[1] += deltaX * deltaY;
    momentSums[2] += deltaY * deltaY;
  }

  const auto sxx = momentSums[0];
  const auto sxy = momentSums[1];
  const auto syy = momentSums[2];

  if (sxx + syy < kEpsilon) {
    return std::nullopt;
  }

  const auto direction = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  const auto normal = direction + (0.5 * std::numbers::pi);
  const auto normalX = std::cos(normal);
  const auto normalY = std::sin(normal);
  const auto rho = ((normalX * meanX) + (normalY * meanY));

  auto model = toLineModel(LineModel{.rho = rho, .alpha = normal});

  const auto lineNormalX = std::cos(model.alpha);
  const auto lineNormalY = std::sin(model.alpha);
  double mse = 0.0;
  for (const auto &point : points) {
    const auto distance = ((lineNormalX * point.x) + (lineNormalY * point.y)) - model.rho;
    mse += distance * distance;
  }
  mse /= count;

  return LineFit{.model = model, .alphaRaw = normal, .pointCount = points.size(), .mse = mse};
}

auto makeExpectedLine(const LineModel &mapLine, const types::Pose &pose) -> LineObservation {
  const auto normalX = std::cos(mapLine.alpha);
  const auto normalY = std::sin(mapLine.alpha);
  const auto rawRho = mapLine.rho - (normalX * pose.x) - (normalY * pose.y);
  const auto rawAlpha = normalizeAngle(mapLine.alpha - pose.theta);

  auto rhoSign = 1.0;
  auto rho = rawRho;
  auto alpha = rawAlpha;
  if (rho < 0.0) {
    rhoSign = -1.0;
    rho = -rho;
    alpha = normalizeAngle(alpha + std::numbers::pi);
  }

  return LineObservation{.observed = LineModel{.rho = 0.0, .alpha = 0.0},
                         .expected = LineModel{.rho = rho, .alpha = alpha},
                         .nx = normalX,
                         .ny = normalY,
                         .rhoSign = rhoSign,
                         .rangeVariance = 0.0,
                         .angleVariance = 0.0};
}

auto gateLineObservation(const LineObservation &observation,
                         const ObservationGateConfig &gateConfig) -> bool {
  const auto h00 = -observation.rhoSign * observation.nx;
  const auto h01 = -observation.rhoSign * observation.ny;

  const auto &covariance = gateConfig.covariance;
  const auto p00 = covariance(0, 0);
  const auto p01 = covariance(0, 1);
  const auto p02 = covariance(0, 2);
  const auto p10 = covariance(1, 0);
  const auto p11 = covariance(1, 1);
  const auto p12 = covariance(1, 2);
  const auto p22 = covariance(2, 2);

  const auto s00 =
      (h00 * (p00 * h00 + p01 * h01)) + (h01 * (p10 * h00 + p11 * h01)) + observation.rangeVariance;
  const auto s01 = (-h00 * p02) - (h01 * p12);
  const auto s11 = p22 + observation.angleVariance;

  const auto det = (s00 * s11) - (s01 * s01);
  if (std::abs(det) < kGateEpsilon) {
    return false;
  }

  const auto invDet = 1.0 / det;
  const auto inv00 = s11 * invDet;
  const auto inv01 = -s01 * invDet;
  const auto inv11 = s00 * invDet;

  const auto residualRho = observation.observed.rho - observation.expected.rho;
  const auto residualAlpha =
      normalizeAngle(observation.observed.alpha - observation.expected.alpha);
  const auto maha = (residualRho * (inv00 * residualRho + inv01 * residualAlpha)) +
                    (residualAlpha * (inv01 * residualRho + inv11 * residualAlpha));
  return maha <= gateConfig.threshold;
}

auto mapSignatureFromMap(const types::MapData &map) -> Result<MapSignature> {
  if (!mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Map dimensions must be positive."});
  }

  return MapSignature{.width = map.width,
                      .height = map.height,
                      .resolution = map.resolution,
                      .gridSize = map.grid.size()};
}

auto signatureMatches(const MapSignature &signature, const types::MapData &map) -> bool {
  return signature.width == map.width && signature.height == map.height &&
         signature.resolution == map.resolution && signature.gridSize == map.grid.size();
}

auto extractLinesFromMap(const types::MapData &map, const HoughConfig &config)
    -> Result<std::vector<MapLine>> {
  if (!mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (config.thetaBins < 2 || config.rhoBins < 2 || config.minVotes <= 0 || config.maxLines <= 0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Hough configuration is invalid."});
  }

  const auto points = collectOccupiedPoints(map);
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

  auto lines = std::vector<MapLine>{};
  for (const auto &candidate : candidates) {
    if (static_cast<int>(lines.size()) >= config.maxLines) {
      break;
    }

    const auto normalized = toLineModel(LineModel{.rho = candidate.rho, .alpha = candidate.alpha});
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

} // namespace ad::localization::util
