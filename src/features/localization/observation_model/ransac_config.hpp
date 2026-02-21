#pragma once

#include <cstddef>

namespace ad::localization {

struct RansacConfig {
  const int maxIterations;
  const std::size_t minInliers;
  const double minInlierRatio;
};

} // namespace ad::localization
