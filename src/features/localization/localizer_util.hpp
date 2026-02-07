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

[[nodiscard]] auto normalizeAngle(double angle) -> double;
[[nodiscard]] auto mapHasConsistentGrid(const types::MapData &map) -> bool;
[[nodiscard]] auto collectOccupiedPoints(const types::MapData &map) -> std::vector<types::Point>;
[[nodiscard]] auto passesGate(double residual, double variance, double threshold) -> bool;
[[nodiscard]] auto toLineModel(LineModel raw) -> LineModel;
[[nodiscard]] auto fitLine(const std::vector<types::Point> &points) -> std::optional<LineFit>;
[[nodiscard]] auto makeExpectedLine(const LineModel &mapLine, double x, double y, double theta)
    -> LineObservation;
[[nodiscard]] auto gateLineObservation(const LineObservation &observation,
                                       const std::array<double, 9> &covariance, double threshold)
    -> bool;
[[nodiscard]] auto mapSignatureFromMap(const types::MapData &map) -> Result<MapSignature>;
[[nodiscard]] auto signatureMatches(const MapSignature &signature, const types::MapData &map)
    -> bool;
[[nodiscard]] auto extractLinesFromMap(const types::MapData &map, const HoughConfig &config)
    -> Result<std::vector<MapLine>>;

} // namespace ad::localization::util
