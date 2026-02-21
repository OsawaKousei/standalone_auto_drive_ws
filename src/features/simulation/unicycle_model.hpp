#pragma once

#include "i_physics.hpp"

namespace ad::simulation {

struct UnicycleModelConfig {
  const double maxLinearSpeed;
  const double maxAngularSpeed;
};

namespace config {

constexpr double kDefaultMaxLinearSpeed = 5.0;
constexpr double kDefaultMaxAngularSpeed = 3.0;

[[nodiscard]] inline auto unicycleDefaultConfig() -> UnicycleModelConfig {
  return UnicycleModelConfig{.maxLinearSpeed = kDefaultMaxLinearSpeed,
                             .maxAngularSpeed = kDefaultMaxAngularSpeed};
}

} // namespace config

class UnicycleModel final : public IPhysicsModel {
public:
  explicit UnicycleModel(UnicycleModelConfig config = config::unicycleDefaultConfig());
  [[nodiscard]] auto propagate(const MotionState &state, const types::Twist &command,
                               double deltaSeconds) const -> Result<MotionResult> override;

private:
  const UnicycleModelConfig config_;
};

} // namespace ad::simulation
