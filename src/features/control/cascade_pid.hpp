#pragma once

#include "i_controller.hpp"

#include <optional>

namespace ad::control {

struct CascadePidConfig {
  const double lookaheadDistance;
  const double maxLinearSpeed;
  const double maxAngularSpeed;
  const double positionKp;
  const double positionKi;
  const double positionKd;
  const double velocityKp;
  const double velocityKi;
  const double velocityKd;
  const double headingKp;
  const double headingKi;
  const double headingKd;
};

class CascadePidController final : public IController {
public:
  explicit CascadePidController(CascadePidConfig config);
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

  [[nodiscard]] static auto selectLookaheadTarget(std::span<const types::Point> path,
                                                  const types::Pose &pose, double lookaheadDistance)
      -> types::Point;

  [[nodiscard]] static auto updatePid(PidState &state, double error, double deltaSeconds,
                                      const PidGains &gains, double integralLimit) -> double;

  const CascadePidConfig config_;
  mutable PidState positionXState_{};
  mutable PidState positionYState_{};
  mutable PidState velocityXState_{};
  mutable PidState velocityYState_{};
  mutable PidState headingState_{};
  mutable PidState yawRateState_{};
  mutable std::optional<types::Pose> previousPose_{};
  mutable std::optional<types::Twist> previousCommand_{};
};

} // namespace ad::control
