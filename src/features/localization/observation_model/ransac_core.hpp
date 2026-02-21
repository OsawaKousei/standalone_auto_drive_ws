#pragma once

#include "../localizer_util.hpp"
#include "hough_ransac_config.hpp"

#include "../../../shared/types.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace ad::localization::ransac {

struct GenericRansacConfig {
  int maxIterations;
  std::size_t sampleSize;
  std::size_t minInliers;
  double minInlierRatio;
};

struct GenericRansacResult {
  std::vector<std::size_t> inlierIndices;
};

using InlierIndexSelector = std::function<std::optional<std::vector<std::size_t>>(
    const std::vector<types::Point> &points, const std::vector<std::size_t> &sampleIndices)>;

[[nodiscard]] auto
runGenericPointRansac(const std::vector<types::Point> &points, const GenericRansacConfig &config,
                      const InlierIndexSelector &selector, std::uint32_t randomSeed = 0U)
    -> std::optional<GenericRansacResult>;

struct RansacLineFitResult {
  util::LineFit fit;
  std::size_t inlierCount;
};

[[nodiscard]] auto fitLineToPoints(const std::vector<types::Point> &points,
                                   const RansacConfig &config, std::uint32_t randomSeed = 0U)
    -> std::optional<RansacLineFitResult>;

} // namespace ad::localization::ransac
