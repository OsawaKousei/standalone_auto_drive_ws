#pragma once

#include "i_planner.hpp"

namespace ad::planning {

class DijkstraPlanner final : public IPlanner {
public:
  DijkstraPlanner() = default;

  [[nodiscard]] auto plan(const types::MapData &map, const types::Pose &start,
                          const types::Pose &goal) const -> Result<types::Path> override;
};

} // namespace ad::planning
