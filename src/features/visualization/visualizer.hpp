#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <memory>
#include <span>
#include <string_view>
#include <utility>

namespace ad::visualization {

// Window presentation and destruction must run on the same thread.
class Visualizer {
  struct Backend;
  struct MapStorage;

public:
  Visualizer();
  ~Visualizer();
  Visualizer(const Visualizer &) = delete;
  auto operator=(const Visualizer &) -> Visualizer & = delete;
  Visualizer(Visualizer &&) noexcept;
  auto operator=(Visualizer &&) noexcept -> Visualizer &;

  struct MapGeometry {
    const int width;
    const int height;
    const double resolution;
  };

  struct PreparedMap {
    const MapGeometry geometry;

  private:
    PreparedMap(MapGeometry geometry, std::shared_ptr<const MapStorage> storage)
        : geometry(geometry), storage_(std::move(storage)) {}
    std::shared_ptr<const MapStorage> storage_;
    friend class Visualizer;
  };

  [[nodiscard]] static auto prepareMap(const types::MapData &map) -> Result<PreparedMap>;
  [[nodiscard]] auto renderFrame(const PreparedMap &prepared) const -> Status;
  [[nodiscard]] auto renderPath(std::span<const types::Point> path, const MapGeometry &map) const
      -> Status;
  [[nodiscard]] auto renderPath(std::span<const types::Point> path, const MapGeometry &map,
                                std::string_view style) const -> Status;
  [[nodiscard]] auto renderRobot(const types::Pose &pose, const types::Footprint &footprint,
                                 const MapGeometry &map) const -> Status;
  [[nodiscard]] auto renderMarker(const types::Point &point, const MapGeometry &map, double size,
                                  std::string_view color) const -> Status;
  [[nodiscard]] auto renderPoints(std::span<const types::Point> points, const MapGeometry &map,
                                  double size, std::string_view color) const -> Status;
  [[nodiscard]] auto renderScan(const types::Pose &pose, const types::LidarScan &scan,
                                const MapGeometry &map) const -> Status;
  [[nodiscard]] auto presentFrame() const -> Status;
  [[nodiscard]] auto saveFigure(std::string_view path) const -> Status;

private:
  [[nodiscard]] auto ensureBackend() const -> Status;
  mutable std::unique_ptr<Backend> backend_;
};

} // namespace ad::visualization
