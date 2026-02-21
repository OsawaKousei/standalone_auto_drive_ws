#pragma once

#include "i_controller.hpp"

#include <cstddef>
#include <optional>

namespace ad::control {

struct PidConfig {
  const double lookaheadDistance;
  const double maxLinearSpeed;
  const double maxAngularSpeed;
  const double positionKp;
  const double positionKi;
  const double positionKd;
  const double headingKp;
  const double headingKi;
  const double headingKd;
};

class PidController final : public IController {
public:
  explicit PidController(PidConfig config);
  [[nodiscard]] auto computeCommand(const ControlInput &input) const
      -> Result<types::Twist> override;

private:
  struct PidGains {
    double proportional;
    double integral;
    double derivative;
  };

  struct PidState {
    double integral{0.0};
    double previousError{0.0};
    bool initialized{false};
  };

  struct LookaheadSelection {
    types::Point target;
    std::size_t closestIndex;
  };

  [[nodiscard]] static auto selectLookaheadTarget(std::span<const types::Point> path,
                                                  const types::Pose &pose, double lookaheadDistance,
                                                  std::size_t minClosestIndex)
      -> LookaheadSelection;

  [[nodiscard]] static auto updatePid(PidState &state, double error, double deltaSeconds,
                                      const PidGains &gains, double integralLimit) -> double;

  const PidConfig config_;
  mutable PidState positionXState_{};
  mutable PidState positionYState_{};
  mutable PidState headingState_{};
  mutable std::optional<types::Twist> previousCommand_{};
  mutable std::size_t pathProgressIndex_{0U};
  mutable std::size_t previousPathSize_{0U};
};

} // namespace ad::control
