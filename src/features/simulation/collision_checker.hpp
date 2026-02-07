#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

namespace ad::simulation {

struct CollisionCheckConfig {
  const double maxTranslationStep;
  const double maxRotationStep;
};

class CollisionChecker final {
public:
  CollisionChecker(const types::MapData &map, const types::Footprint &footprint,
                   CollisionCheckConfig config);

  [[nodiscard]] auto checkTrajectory(const types::Pose &start, const types::Pose &end) const
      -> Result<bool>;
  [[nodiscard]] auto isPoseCollisionFree(const types::Pose &pose) const -> Result<bool>;

private:
  const types::MapData &map_;
  const types::Footprint &footprint_;
  const CollisionCheckConfig config_;
};

} // namespace ad::simulation
