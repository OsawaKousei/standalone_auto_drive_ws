#include "astar_planner.hpp"

#include <cstddef>

namespace ad::planning {

auto AStarPlanner::plan(const types::MapData &map, const types::Pose &start,
                        const types::Pose &goal) const -> Result<types::Path> {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  if (map.grid.size() != expectedCells) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  const types::Path path = {{start.x, start.y}, {goal.x, goal.y}};
  return path;
}

} // namespace ad::planning
