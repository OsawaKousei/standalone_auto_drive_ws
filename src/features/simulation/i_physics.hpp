#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

namespace ad::simulation {

struct MotionState {
  const types::Pose pose;
  const types::Twist twist;
};

struct MotionResult {
  const types::Pose pose;
  const types::Twist twist;
};

class IPhysicsModel {
public:
  virtual ~IPhysicsModel() = default;
  [[nodiscard]] virtual Result<MotionResult>
  propagate(const MotionState &state, const types::Twist &command, double deltaSeconds) const = 0;
};

} // namespace ad::simulation
