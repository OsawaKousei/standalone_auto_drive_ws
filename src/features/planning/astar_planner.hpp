#pragma once

#include "i_planner.hpp"

namespace ad::planning {

class AStarPlanner final : public IPlanner {
public:
  AStarPlanner() = default;
  [[nodiscard]] auto plan(const types::MapData &map, const types::Pose &start,
                          const types::Pose &goal) const -> Result<types::Path> override;
};

} // namespace ad::planning
