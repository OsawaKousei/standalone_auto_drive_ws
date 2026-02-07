#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <matplotlibcpp.h>

#include <optional>
#include <span>

namespace ad::visualization {

class Visualizer {
public:
  Visualizer() = default;
  [[nodiscard]] auto renderFrame(const types::MapData &map) const -> Status;
  [[nodiscard]] auto renderPath(std::span<const types::Point> path) const -> Status;
  [[nodiscard]] auto renderRobot(const types::Pose &pose, const types::Footprint &footprint) const
      -> Status;
  [[nodiscard]] auto renderScan(const types::Pose &pose, std::span<const double> ranges) const
      -> Status;

private:
  struct MapGeometry {
    int width;
    int height;
    double resolution;
  };

  [[nodiscard]] static auto configurePythonEnvironment() -> Status;
  [[nodiscard]] auto currentMapGeometry() const -> Result<MapGeometry>;

  mutable std::optional<MapGeometry> last_map_{};
};

} // namespace ad::visualization
