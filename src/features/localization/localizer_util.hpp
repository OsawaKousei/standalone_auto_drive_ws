#pragma once

#include "localization_config.hpp"
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
  double alphaRaw;
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

struct GateCheck {
  double residual;
  double variance;
  double threshold;
};

struct ObservationGateConfig {
  const CovarianceMatrix &covariance;
  double threshold;
};

[[nodiscard]] auto normalizeAngle(double angle) -> double;
[[nodiscard]] auto mapHasConsistentGrid(const types::MapData &map) -> bool;
[[nodiscard]] auto collectOccupiedPoints(const types::MapData &map) -> std::vector<types::Point>;
[[nodiscard]] auto passesGate(const GateCheck &check) -> bool;
[[nodiscard]] auto toLineModel(LineModel raw) -> LineModel;
[[nodiscard]] auto fitLine(const std::vector<types::Point> &points) -> std::optional<LineFit>;
[[nodiscard]] auto makeExpectedLine(const LineModel &mapLine, const types::Pose &pose)
    -> LineObservation;
[[nodiscard]] auto gateLineObservation(const LineObservation &observation,
                                       const ObservationGateConfig &gateConfig) -> bool;
[[nodiscard]] auto mapSignatureFromMap(const types::MapData &map) -> Result<MapSignature>;
[[nodiscard]] auto signatureMatches(const MapSignature &signature, const types::MapData &map)
    -> bool;

} // namespace ad::localization::util
