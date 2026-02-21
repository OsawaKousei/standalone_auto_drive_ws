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
};

struct EkfLocalizerConfig {
  const EkfConfig ekf;
};

using CovarianceMatrix = Eigen::Matrix3d;

} // namespace ad::localization
