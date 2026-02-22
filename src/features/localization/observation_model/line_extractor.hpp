#pragma once

#include "../localizer_util.hpp"

#include "../../../shared/result.hpp"
#include "../../../shared/types.hpp"

#include <vector>

namespace ad::localization::line_extractor {

[[nodiscard]] auto extractMapLinesFromMap(const types::MapData &map,
                                          const MapLineExtractionConfig &config)
    -> Result<std::vector<util::MapLine>>;

[[nodiscard]] auto extractMapLinesFromMap(const types::MapData &map)
    -> Result<std::vector<util::MapLine>>;

} // namespace ad::localization::line_extractor
