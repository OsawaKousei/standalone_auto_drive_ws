#pragma once

#include "i_controller.hpp"

namespace ad::control {

struct PurePursuitConfig {
  const double lookaheadDistance;
  const double desiredLinearVelocity;
};

namespace config {

constexpr double kDefaultLookaheadDistance = 0.6;
constexpr double kDefaultDesiredLinearVelocity = 1.2;

[[nodiscard]] inline auto purePursuitDefaultConfig() -> PurePursuitConfig {
  return PurePursuitConfig{.lookaheadDistance = kDefaultLookaheadDistance,
                           .desiredLinearVelocity = kDefaultDesiredLinearVelocity};
}

} // namespace config

class PurePursuitController final : public IController {
public:
  explicit PurePursuitController(PurePursuitConfig config);
  [[nodiscard]] auto computeCommand(const ControlInput &input) const
      -> Result<types::Twist> override;

private:
  const PurePursuitConfig config_;
};

} // namespace ad::control
