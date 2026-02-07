#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <matplotlibcpp.h>

#include <span>
#include <string_view>
#include <vector>

namespace ad::visualization {

class Visualizer {
public:
  Visualizer() = default;

  struct MapGeometry {
    const int width;
    const int height;
    const double resolution;
  };

  struct PreparedMap {
    const MapGeometry geometry;
    const std::vector<float> gridImage;
  };

  [[nodiscard]] static auto prepareMap(const types::MapData &map) -> Result<PreparedMap>;
  [[nodiscard]] auto renderFrame(const PreparedMap &prepared) const -> Status;
  [[nodiscard]] auto renderPath(std::span<const types::Point> path, const MapGeometry &map) const
      -> Status;
  [[nodiscard]] auto renderRobot(const types::Pose &pose, const types::Footprint &footprint,
                                 const MapGeometry &map) const -> Status;
  [[nodiscard]] auto renderScan(const types::Pose &pose, std::span<const double> ranges,
                                const MapGeometry &map) const -> Status;
  [[nodiscard]] auto presentFrame() const -> Status;
  [[nodiscard]] auto saveFigure(std::string_view path) const -> Status;

private:
  [[nodiscard]] static auto configurePythonEnvironment() -> Status;
};

} // namespace ad::visualization
