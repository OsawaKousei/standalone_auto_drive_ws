#pragma once

#include "hough_ransac_config.hpp"
#include "localizer_util.hpp"

#include "../../shared/types.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace ad::localization::ransac {

struct RansacLineFitResult {
  util::LineFit fit;
  std::size_t inlierCount;
};

[[nodiscard]] auto fitLineToPoints(const std::vector<types::Point> &points,
                                   const RansacConfig &config, std::uint32_t randomSeed = 0U)
    -> std::optional<RansacLineFitResult>;

} // namespace ad::localization::ransac
