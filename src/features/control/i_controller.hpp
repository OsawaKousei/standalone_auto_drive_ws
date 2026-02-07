#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <span>

namespace ad::control {

struct ControlInput {
  const std::span<const types::Point> path;
  const types::Pose currentPose;
};

class IController {
public:
  virtual ~IController() = default;
  [[nodiscard]] virtual Result<types::Twist> computeCommand(const ControlInput &input) const = 0;
};

} // namespace ad::control
