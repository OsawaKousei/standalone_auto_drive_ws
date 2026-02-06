#include "lidar_sim.hpp"

#include <algorithm>
#include <cstddef>

namespace ad::simulation {

auto LidarSim::simulate(const types::MapData &map, const types::Pose &pose) const
    -> Result<LidarScan> {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  if (map.grid.size() != expectedCells) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  const auto rayCount = static_cast<std::size_t>(std::max(map.width, 1));
  const auto maxRange = map.resolution * static_cast<double>(std::max(map.width, map.height));

  const LidarScan scan(rayCount, maxRange);
  (void)pose; // pose influences simulation in full implementation
  return scan;
}

} // namespace ad::simulation
