#pragma once

#include "../../../shared/result.hpp"
#include "../../../shared/types.hpp"
#include "../i_observation_model.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace ad::localization::observation_model::util {

struct LineModel {
  double rho;
  double alpha;
};

struct MapLine {
  types::LineSegment segment;
  LineModel model;
  double directionX;
  double directionY;
  double minProjection;
  double maxProjection;
};

struct LineObservation {
  LineModel observed;
  LineModel expected;
  double nx;
  double ny;
  double rhoSign;
  double rangeVariance;
  double angleVariance;
};

struct MapSignature {
  int width;
  int height;
  double resolution;
  std::size_t gridSize;
};

struct ObservationGateConfig {
  const CovarianceMatrix &covariance;
  double threshold;
};

struct ObservationNoiseConfig {
  double measurementNoiseRange;
  double measurementNoiseAngle;
};

[[nodiscard]] auto mapHasConsistentGrid(const types::MapData &map) -> bool;
[[nodiscard]] auto toLineModel(LineModel raw) -> LineModel;
[[nodiscard]] auto makeExpectedLine(const LineModel &mapLine, const types::Pose &pose)
    -> LineObservation;
[[nodiscard]] auto gateLineObservation(const LineObservation &observation,
                                       const ObservationGateConfig &gateConfig) -> bool;
auto applyObservationNoiseFromMse(LineObservation &observation,
                                  const ObservationNoiseConfig &config, double supportPointCount,
                                  double mse) -> void;
auto buildMeasurementData(const std::vector<LineObservation> &observations, double score)
    -> ObservationUpdateInput;
[[nodiscard]] auto mapSignatureFromMap(const types::MapData &map) -> Result<MapSignature>;
[[nodiscard]] auto signatureMatches(const MapSignature &signature, const types::MapData &map)
    -> bool;

} // namespace ad::localization::observation_model::util
