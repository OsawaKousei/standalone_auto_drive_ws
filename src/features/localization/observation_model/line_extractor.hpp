#pragma once

#include "line_observation_util.hpp"

#include "../../../shared/result.hpp"
#include "../../../shared/types.hpp"

#include <vector>

namespace ad::localization::line_extractor {

struct MapLineExtractionConfig {
  const int maxLines;
  const double minSegmentLength;
};

[[nodiscard]] auto extractMapLinesFromMap(const types::MapData &map,
                                          const MapLineExtractionConfig &config)
    -> Result<std::vector<observation_model::util::MapLine>>;

[[nodiscard]] auto extractMapLinesFromMap(const types::MapData &map)
    -> Result<std::vector<observation_model::util::MapLine>>;

} // namespace ad::localization::line_extractor
