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
  IPhysicsModel() = default;
  IPhysicsModel(const IPhysicsModel &) = delete;
  auto operator=(const IPhysicsModel &) -> IPhysicsModel & = delete;
  IPhysicsModel(IPhysicsModel &&) = delete;
  auto operator=(IPhysicsModel &&) -> IPhysicsModel & = delete;
  [[nodiscard]] virtual auto propagate(const MotionState &state, const types::Twist &command,
                                       double deltaSeconds) const -> Result<MotionResult> = 0;
};

} // namespace ad::simulation
