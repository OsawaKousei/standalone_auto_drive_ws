#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <cstddef>

namespace ad::planning::utils {

struct GridCoord {
  const int x;
  const int y;
};

struct GridOffset {
  const int dx;
  const int dy;
};

[[nodiscard]] auto cellCount(const types::MapData &map) -> std::size_t;
[[nodiscard]] auto isValidMap(const types::MapData &map) -> Result<void>;
[[nodiscard]] auto toIndex(const types::MapData &map, const GridCoord &coord) -> std::size_t;
[[nodiscard]] auto toCoord(const types::MapData &map, std::size_t index) -> GridCoord;
[[nodiscard]] auto cellCenter(const types::MapData &map, const GridCoord &coord) -> types::Point;
[[nodiscard]] auto worldToCell(const types::MapData &map, const types::Pose &pose)
    -> Result<GridCoord>;
[[nodiscard]] auto isObstacle(const types::MapData &map, const GridCoord &coord) -> bool;

} // namespace ad::planning::utils
