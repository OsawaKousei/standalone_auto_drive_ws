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

namespace ad::localization::config {

[[nodiscard]] inline auto ekfLocalizerDefaultConfig() -> EkfLocalizerConfig {
  return EkfLocalizerConfig{.hough = HoughConfig{.thetaBins = 180,
                                                 .rhoBins = 200,
                                                 .minVotes = 25,
                                                 .maxLines = 40,
                                                 .inlierDistance = 0.12,
                                                 .minSegmentLength = 0.8,
                                                 .mergeRho = 0.2,
                                                 .mergeTheta = 0.08},
                            .ekf = EkfConfig{.processNoiseTranslation = 0.05,
                                             .processNoiseRotation = 0.03,
                                             .measurementNoiseRange = 0.12,
                                             .measurementNoiseAngle = 0.12},
                            .maxAssociationDistance = 0.3,
                            .segmentMargin = 0.3,
                            .gateThreshold = 6.0,
                            .minObservations = 3U};
}

} // namespace ad::localization::config
