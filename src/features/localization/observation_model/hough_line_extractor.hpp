#pragma once

#include "../localization_config.hpp"
#include "../localizer_util.hpp"

#include "../../../shared/result.hpp"
#include "../../../shared/types.hpp"

#include <vector>

namespace ad::localization::hough {

[[nodiscard]] auto extractMapLinesFromMap(const types::MapData &map, const HoughConfig &config)
    -> Result<std::vector<util::MapLine>>;

[[nodiscard]] auto extractLinesFromPoints(const std::vector<types::Point> &points,
                                          const HoughConfig &config)
    -> Result<std::vector<util::MapLine>>;

} // namespace ad::localization::hough
