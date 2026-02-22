#pragma once

#include "../i_physics.hpp"

namespace ad::simulation {

struct HolonomicModelConfig {
  const double maxLinearSpeedX;
  const double maxLinearSpeedY;
  const double maxAngularSpeed;
  const double maxLinearAccelerationX;
  const double maxLinearAccelerationY;
  const double maxAngularAcceleration;
  const double tauLinearX;
  const double tauLinearY;
  const double tauAngular;
};

class HolonomicModel final : public IPhysicsModel {
public:
  explicit HolonomicModel(HolonomicModelConfig config);
  [[nodiscard]] auto propagate(const MotionState &state, const types::Twist &command,
                               double deltaSeconds) const -> Result<MotionResult> override;

private:
  const HolonomicModelConfig config_;
};

} // namespace ad::simulation
