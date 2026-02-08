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

constexpr int kDefaultThetaBins = 180;
constexpr int kDefaultRhoBins = 200;
constexpr int kDefaultMinVotes = 25;
constexpr int kDefaultMaxLines = 40;
constexpr double kDefaultInlierDistance = 0.12;
constexpr double kDefaultMinSegmentLength = 0.8;
constexpr double kDefaultMergeRho = 0.2;
constexpr double kDefaultMergeTheta = 0.08;

constexpr double kDefaultProcessNoiseTranslation = 0.05;
constexpr double kDefaultProcessNoiseRotation = 0.03;
constexpr double kDefaultMeasurementNoiseRange = 0.12;
constexpr double kDefaultMeasurementNoiseAngle = 0.12;

constexpr double kDefaultMaxAssociationDistance = 0.3;
constexpr double kDefaultSegmentMargin = 0.3;
constexpr double kDefaultGateThreshold = 6.0;
constexpr std::size_t kDefaultMinObservations = 3U;

[[nodiscard]] inline auto ekfLocalizerDefaultConfig() -> EkfLocalizerConfig {
  return EkfLocalizerConfig{
      .hough = HoughConfig{.thetaBins = kDefaultThetaBins,
                           .rhoBins = kDefaultRhoBins,
                           .minVotes = kDefaultMinVotes,
                           .maxLines = kDefaultMaxLines,
                           .inlierDistance = kDefaultInlierDistance,
                           .minSegmentLength = kDefaultMinSegmentLength,
                           .mergeRho = kDefaultMergeRho,
                           .mergeTheta = kDefaultMergeTheta},
      .ekf = EkfConfig{.processNoiseTranslation = kDefaultProcessNoiseTranslation,
                       .processNoiseRotation = kDefaultProcessNoiseRotation,
                       .measurementNoiseRange = kDefaultMeasurementNoiseRange,
                       .measurementNoiseAngle = kDefaultMeasurementNoiseAngle},
      .maxAssociationDistance = kDefaultMaxAssociationDistance,
      .segmentMargin = kDefaultSegmentMargin,
      .gateThreshold = kDefaultGateThreshold,
      .minObservations = kDefaultMinObservations};
}

} // namespace ad::localization::config
