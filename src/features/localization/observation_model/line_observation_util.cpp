#include "line_observation_util.hpp"

#include "../localizer_util.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ad::localization::observation_model::util {

namespace {

constexpr double kGateEpsilon = 1e-12;
constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;

} // namespace

auto mapHasConsistentGrid(const types::MapData &map) -> bool {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.grid.size() == expectedCells;
}

auto toLineModel(LineModel raw) -> LineModel {
  auto normalizedRho = raw.rho;
  auto normalizedAlpha = ad::localization::util::normalizeAngle(raw.alpha);
  if (normalizedRho < 0.0) {
    normalizedRho = -normalizedRho;
    normalizedAlpha = ad::localization::util::normalizeAngle(normalizedAlpha + std::numbers::pi);
  }
  return LineModel{.rho = normalizedRho, .alpha = normalizedAlpha};
}

auto makeExpectedLine(const LineModel &mapLine, const types::Pose &pose) -> LineObservation {
  const auto normalX = std::cos(mapLine.alpha);
  const auto normalY = std::sin(mapLine.alpha);
  const auto rawRho = mapLine.rho - (normalX * pose.x) - (normalY * pose.y);
  const auto rawAlpha = ad::localization::util::normalizeAngle(mapLine.alpha - pose.theta);

  auto rhoSign = 1.0;
  auto rho = rawRho;
  auto alpha = rawAlpha;
  if (rho < 0.0) {
    rhoSign = -1.0;
    rho = -rho;
    alpha = ad::localization::util::normalizeAngle(alpha + std::numbers::pi);
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
  const auto residualAlpha = ad::localization::util::normalizeAngle(observation.observed.alpha -
                                                                    observation.expected.alpha);
  const auto maha = (residualRho * (inv00 * residualRho + inv01 * residualAlpha)) +
                    (residualAlpha * (inv01 * residualRho + inv11 * residualAlpha));
  return maha <= gateConfig.threshold;
}

auto applyObservationNoiseFromMse(LineObservation &observation,
                                  const ObservationNoiseConfig &config,
                                  const double supportPointCount, const double mse) -> void {
  const auto pointCount = std::max(1.0, supportPointCount);
  const auto baseRangeVar = config.measurementNoiseRange * config.measurementNoiseRange;
  const auto baseAngleVar = config.measurementNoiseAngle * config.measurementNoiseAngle;
  const auto scale = std::max(1.0, kReferencePoints / pointCount);
  const auto minRangeVar = baseRangeVar * kMinRangeVarianceFactor;
  const auto minAngleVar = baseAngleVar * kMinAngleVarianceFactor;

  observation.rangeVariance = std::max(minRangeVar, (baseRangeVar * scale) + mse);
  observation.angleVariance =
      std::max(minAngleVar, (baseAngleVar * scale) + (mse * kAngleMseScale));
}

auto buildMeasurementData(const std::vector<LineObservation> &observations, const double score)
    -> ObservationUpdateInput {
  ObservationUpdateInput data{};
  const auto measurementCount = observations.size() * 2U;
  const auto size = static_cast<Eigen::Index>(measurementCount);
  data.residual = Eigen::VectorXd::Zero(size);
  data.measurementMatrix = Eigen::MatrixXd::Zero(size, 3);
  data.measurementNoise = Eigen::MatrixXd::Zero(size, size);
  data.score = score;

  for (std::size_t index = 0; index < observations.size(); ++index) {
    const auto &obs = observations[index];
    const auto row = static_cast<Eigen::Index>(index * 2U);

    const auto residualRho = obs.observed.rho - obs.expected.rho;
    const auto residualAlpha =
        ad::localization::util::normalizeAngle(obs.observed.alpha - obs.expected.alpha);
    data.residual(row) = residualRho;
    data.residual(row + 1) = residualAlpha;

    const auto hRhoX = -obs.rhoSign * obs.nx;
    const auto hRhoY = -obs.rhoSign * obs.ny;

    data.measurementMatrix(row, 0) = hRhoX;
    data.measurementMatrix(row, 1) = hRhoY;
    data.measurementMatrix(row, 2) = 0.0;

    data.measurementMatrix(row + 1, 0) = 0.0;
    data.measurementMatrix(row + 1, 1) = 0.0;
    data.measurementMatrix(row + 1, 2) = -1.0;

    data.measurementNoise(row, row) = obs.rangeVariance;
    data.measurementNoise(row + 1, row + 1) = obs.angleVariance;
  }

  return data;
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

} // namespace ad::localization::observation_model::util
