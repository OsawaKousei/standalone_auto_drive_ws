#pragma once

#include "i_physics.hpp"

namespace ad::simulation {

struct UnicycleModelConfig {
  const double maxLinearSpeed;
  const double maxAngularSpeed;
  const double maxLinearAcceleration;
  const double maxAngularAcceleration;
  const double tauLinear;
  const double tauAngular;
};

class UnicycleModel final : public IPhysicsModel {
public:
  explicit UnicycleModel(UnicycleModelConfig config);
  [[nodiscard]] auto propagate(const MotionState &state, const types::Twist &command,
                               double deltaSeconds) const -> Result<MotionResult> override;

private:
  const UnicycleModelConfig config_;
};

} // namespace ad::simulation
