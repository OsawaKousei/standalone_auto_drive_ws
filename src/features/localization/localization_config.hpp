#pragma once

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

} // namespace ad::localization
