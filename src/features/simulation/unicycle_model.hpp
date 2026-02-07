#pragma once

#include "i_physics.hpp"

namespace ad::simulation {

class UnicycleModel final : public IPhysicsModel {
public:
  UnicycleModel() = default;
  [[nodiscard]] auto propagate(const MotionState &state, const types::Twist &command,
                               double deltaSeconds) const -> Result<MotionResult> override;
};

} // namespace ad::simulation
