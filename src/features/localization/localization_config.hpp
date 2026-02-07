#pragma once

namespace ad::localization {

struct EkfConfig {
  const double processNoiseTranslation;
  const double processNoiseRotation;
  const double measurementNoiseRange;
  const double measurementNoiseAngle;
};

} // namespace ad::localization
