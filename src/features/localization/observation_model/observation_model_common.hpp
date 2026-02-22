#pragma once

#include "../i_observation_model.hpp"
#include "../localizer_util.hpp"

#include <vector>

namespace ad::localization::observation_model_common {

struct ObservationNoiseConfig {
  double measurementNoiseRange;
  double measurementNoiseAngle;
};

auto applyObservationNoiseFromMse(util::LineObservation &observation,
                                  const ObservationNoiseConfig &config, double supportPointCount,
                                  double mse) -> void;

auto applyObservationNoiseFromResidual(util::LineObservation &observation,
                                       const ObservationNoiseConfig &config, double angleResidual,
                                       double rhoResidual) -> void;

auto buildMeasurementData(const std::vector<util::LineObservation> &observations, double score)
    -> ObservationUpdateInput;

} // namespace ad::localization::observation_model_common
