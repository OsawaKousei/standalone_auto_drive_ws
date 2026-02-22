#pragma once

#include "../localizer_util.hpp"

#include <optional>

namespace ad::localization::line_observation_builder {

struct ObservationBuildConfig {
  double measurementNoiseRange;
  double measurementNoiseAngle;
  double gateThreshold;
};

[[nodiscard]] auto
buildGatedObservationFromFit(const util::LineModel &mapLine, const util::LineModel &observedLine,
                             const types::Pose &predictedPose, double supportPointCount, double mse,
                             const ObservationBuildConfig &config,
                             const CovarianceMatrix &covariance)
    -> std::optional<util::LineObservation>;

[[nodiscard]] auto buildGatedObservationFromResidual(const util::LineModel &mapLine,
                                                     const util::LineModel &observedLine,
                                                     const types::Pose &predictedPose,
                                                     double angleResidual, double rhoResidual,
                                                     const ObservationBuildConfig &config,
                                                     const CovarianceMatrix &covariance)
    -> std::optional<util::LineObservation>;

} // namespace ad::localization::line_observation_builder
