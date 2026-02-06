#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <span>

namespace ad::control {

class IController {
public:
  virtual ~IController() = default;
  [[nodiscard]] virtual Result<types::Twist> computeCommand(std::span<const types::Point> path,
                                                            const types::Pose &currentPose,
                                                            double lookaheadDistance) const = 0;
};

} // namespace ad::control
