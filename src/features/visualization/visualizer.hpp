#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <span>

namespace ad::visualization {

class Visualizer {
public:
  Visualizer() = default;
  [[nodiscard]] auto renderPath(std::span<const types::Point> path) const -> Status;
  [[nodiscard]] auto renderPose(const types::Pose &pose) const -> Status;
  [[nodiscard]] auto renderScan(std::span<const double> ranges) const -> Status;
};

} // namespace ad::visualization
