#include "localizer_util.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

namespace ad::localization::util {

namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kGateEpsilon = 1e-12;
constexpr double kTwo = 2.0;
constexpr double kTwoPi = kTwo * std::numbers::pi;

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

  return LineFit{.model = model, .pointCount = points.size(), .mse = mse};
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

} // namespace ad::localization::util
