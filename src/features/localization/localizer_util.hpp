#pragma once

#include "i_observation_model.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace ad::localization::util {

struct LineModel {
  double rho;
  double alpha;
};

struct LineFit {
  LineModel model;
  std::size_t pointCount;
  double mse;
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

[[nodiscard]] auto normalizeAngle(double angle) -> double;
[[nodiscard]] auto mapHasConsistentGrid(const types::MapData &map) -> bool;
[[nodiscard]] auto toLineModel(LineModel raw) -> LineModel;
[[nodiscard]] auto fitLine(const std::vector<types::Point> &points) -> std::optional<LineFit>;
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

} // namespace ad::localization::util
