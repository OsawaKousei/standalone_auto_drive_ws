#pragma once

#include "../i_collision_checker.hpp"
#include "../i_planner.hpp"

#include <memory>

namespace ad::planning {

class AStarPlanner final : public IPlanner {
public:
  explicit AStarPlanner(std::unique_ptr<ICollisionChecker> collisionChecker);

  [[nodiscard]] auto plan(const types::MapData &map, const types::Pose &start,
                          const types::Pose &goal, const types::Footprint &footprint) const
      -> Result<types::Path> override;

private:
  const std::unique_ptr<ICollisionChecker> collisionChecker_;
};

} // namespace ad::planning
