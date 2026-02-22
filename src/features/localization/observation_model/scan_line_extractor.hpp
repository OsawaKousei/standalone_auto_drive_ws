#pragma once

#include "ransac_core.hpp"

#include "../localization_config.hpp"
#include "../localizer_util.hpp"

#include "../../../shared/result.hpp"
#include "../../../shared/types.hpp"

#include <vector>

namespace ad::localization::scan_line_extractor {

struct RansacScanLineExtractionConfig {
  RansacLineExtractionConfig lineExtraction;
  RansacConfig ransac;
};

[[nodiscard]] auto collectScanPoints(const types::LidarScan &scan) -> std::vector<types::Point>;

[[nodiscard]] auto extractLinesFromPointsRansac(const std::vector<types::Point> &points,
                                                const RansacScanLineExtractionConfig &config)
    -> Result<std::vector<util::MapLine>>;

} // namespace ad::localization::scan_line_extractor
