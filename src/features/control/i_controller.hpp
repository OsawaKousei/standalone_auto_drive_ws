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
  IController() = default;
  IController(const IController &) = delete;
  auto operator=(const IController &) -> IController = delete;
  IController(IController &&) = delete;
  auto operator=(IController &&) -> IController = delete;
  [[nodiscard]] virtual auto computeCommand(const ControlInput &input) const
      -> Result<types::Twist> = 0;
};

} // namespace ad::control
