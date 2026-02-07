#pragma once

#include "shared/types.hpp"

#include <vector>

namespace ad::localization::util {

[[nodiscard]] auto normalizeAngle(double angle) -> double;
[[nodiscard]] auto mapHasConsistentGrid(const types::MapData &map) -> bool;
[[nodiscard]] auto collectOccupiedPoints(const types::MapData &map) -> std::vector<types::Point>;
[[nodiscard]] auto passesGate(double residual, double variance, double threshold) -> bool;

} // namespace ad::localization::util
