#pragma once

#include <Eigen/Dense>
#include <cstddef>

namespace ad::localization {

struct HoughConfig {
  const int thetaBins;
  const int rhoBins;
  const int minVotes;
  const int maxLines;
  const double inlierDistance;
  const double minSegmentLength;
  const double mergeRho;
  const double mergeTheta;
};

struct EkfConfig {
  const double processNoiseTranslation;
  const double processNoiseRotation;
  const double measurementNoiseRange;
  const double measurementNoiseAngle;
};

struct EkfLocalizerConfig {
  const HoughConfig hough;
  const EkfConfig ekf;
  const double maxAssociationDistance;
  const double segmentMargin;
  const double gateThreshold;
  const std::size_t minObservations;
};

using CovarianceMatrix = Eigen::Matrix3d;

} // namespace ad::localization
