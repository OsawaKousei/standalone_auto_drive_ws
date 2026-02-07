#pragma once

#include "i_collision_checker.hpp"
#include "i_planner.hpp"

#include <functional>

namespace ad::planning {

class DijkstraPlanner final : public IPlanner {
public:
  explicit DijkstraPlanner(const ICollisionChecker &collisionChecker);

  [[nodiscard]] auto plan(const types::MapData &map, const types::Pose &start,
                          const types::Pose &goal, const types::Footprint &footprint) const
      -> Result<types::Path> override;

private:
  const std::reference_wrapper<const ICollisionChecker> collisionChecker_;
};

} // namespace ad::planning
