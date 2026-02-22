#pragma once

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

} // namespace ad::localization::util
