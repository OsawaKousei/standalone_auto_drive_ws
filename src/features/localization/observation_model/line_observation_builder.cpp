#include "line_observation_builder.hpp"

namespace ad::localization::line_observation_builder {

auto buildGatedObservationFromFit(const util::LineModel &mapLine,
                                  const util::LineModel &observedLine,
                                  const types::Pose &predictedPose, const double supportPointCount,
                                  const double mse, const ObservationBuildConfig &config,
                                  const CovarianceMatrix &covariance)
    -> std::optional<util::LineObservation> {
  auto observation = util::makeExpectedLine(mapLine, predictedPose);
  observation.observed = observedLine;
  util::applyObservationNoiseFromMse(
      observation,
      util::ObservationNoiseConfig{.measurementNoiseRange = config.measurementNoiseRange,
                                   .measurementNoiseAngle = config.measurementNoiseAngle},
      supportPointCount, mse);

  if (!util::gateLineObservation(observation,
                                 util::ObservationGateConfig{.covariance = covariance,
                                                             .threshold = config.gateThreshold})) {
    return std::nullopt;
  }

  return observation;
}

auto buildGatedObservationFromResidual(const util::LineModel &mapLine,
                                       const util::LineModel &observedLine,
                                       const types::Pose &predictedPose, const double angleResidual,
                                       const double rhoResidual,
                                       const ObservationBuildConfig &config,
                                       const CovarianceMatrix &covariance)
    -> std::optional<util::LineObservation> {
  auto observation = util::makeExpectedLine(mapLine, predictedPose);
  observation.observed = observedLine;
  util::applyObservationNoiseFromResidual(
      observation,
      util::ObservationNoiseConfig{.measurementNoiseRange = config.measurementNoiseRange,
                                   .measurementNoiseAngle = config.measurementNoiseAngle},
      angleResidual, rhoResidual);

  if (!util::gateLineObservation(observation,
                                 util::ObservationGateConfig{.covariance = covariance,
                                                             .threshold = config.gateThreshold})) {
    return std::nullopt;
  }

  return observation;
}

} // namespace ad::localization::line_observation_builder
