#pragma once

#include "../i_collision_checker.hpp"

#include <memory>

namespace ad::planning {

class GridCollisionChecker final : public ICollisionChecker {
public:
  [[nodiscard]] static auto create(const types::MapData &map, const types::Footprint &footprint)
      -> Result<std::unique_ptr<ICollisionChecker>>;

  GridCollisionChecker(types::MapData inflatedMap, double footprintRadius);

  [[nodiscard]] auto isFree(const types::Pose &pose, const types::Footprint &footprint) const
      -> Result<bool> override;

private:
  const types::MapData inflatedMap_;
  const double footprintRadius_;
};

} // namespace ad::planning
