#include "observation_model_common.hpp"

#include <algorithm>

namespace {

constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;

} // namespace

namespace ad::localization::observation_model_common {

auto applyObservationNoiseFromMse(util::LineObservation &observation,
                                  const HoughObservationModelConfig &config,
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

auto applyObservationNoiseFromResidual(util::LineObservation &observation,
                                       const HoughObservationModelConfig &config,
                                       const double angleResidual, const double rhoResidual)
    -> void {
  const auto mseLike =
      (rhoResidual * rhoResidual) + (angleResidual * angleResidual * kAngleMseScale);
  applyObservationNoiseFromMse(observation, config, kReferencePoints, mseLike);
}

auto buildMeasurementData(const std::vector<util::LineObservation> &observations,
                          const double score) -> ObservationUpdateInput {
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
    const auto residualAlpha = util::normalizeAngle(obs.observed.alpha - obs.expected.alpha);
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

} // namespace ad::localization::observation_model_common
