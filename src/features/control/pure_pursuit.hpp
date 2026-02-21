#pragma once

#include "i_controller.hpp"

namespace ad::control {

struct PurePursuitConfig {
  const double lookaheadDistance;
  const double desiredLinearVelocity;
};

class PurePursuitController final : public IController {
public:
  explicit PurePursuitController(PurePursuitConfig config);
  [[nodiscard]] auto computeCommand(const ControlInput &input) const
      -> Result<types::Twist> override;

private:
  const PurePursuitConfig config_;
};

} // namespace ad::control
