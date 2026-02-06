#pragma once

#include "i_controller.hpp"

namespace ad::control {

class PurePursuitController final : public IController {
public:
  PurePursuitController() = default;
  [[nodiscard]] auto computeCommand(std::span<const types::Point> path,
                                    const types::Pose &currentPose, double lookaheadDistance) const
      -> Result<types::Twist> override;
};

} // namespace ad::control
