#pragma once

#include "shared/types.hpp"

#include <vector>

namespace ad::localization::util {

[[nodiscard]] auto normalizeAngle(double angle) -> double;
[[nodiscard]] auto mapHasConsistentGrid(const types::MapData &map) -> bool;
[[nodiscard]] auto collectOccupiedPoints(const types::MapData &map) -> std::vector<types::Point>;

} // namespace ad::localization::util
