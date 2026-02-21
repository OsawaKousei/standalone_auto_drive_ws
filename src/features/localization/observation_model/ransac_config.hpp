#pragma once

#include <cstddef>

namespace ad::localization {

struct RansacConfig {
  const int maxIterations;
  const double inlierDistance;
  const std::size_t minInliers;
  const double minInlierRatio;
  const double minInlierSpan;
};

} // namespace ad::localization
