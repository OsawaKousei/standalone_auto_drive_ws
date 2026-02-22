#pragma once

#include "../localization_config.hpp"
#include "../localizer_util.hpp"

#include "../../../shared/result.hpp"
#include "../../../shared/types.hpp"

#include <vector>

namespace ad::localization::line_extractor {

[[nodiscard]] auto extractMapLinesFromMap(const types::MapData &map, const HoughConfig &config)
    -> Result<std::vector<util::MapLine>>;

} // namespace ad::localization::line_extractor
