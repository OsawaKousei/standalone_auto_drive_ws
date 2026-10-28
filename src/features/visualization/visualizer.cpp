#include "visualizer.hpp"

#include <ad_visualization_ffi.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace ad::visualization {
namespace {

struct VisualizerDeleter {
  void operator()(AdVisualizer *handle) const noexcept { ad_viz_destroy(handle); }
};
struct MapDeleter {
  void operator()(AdPreparedMap *handle) const noexcept { ad_viz_prepared_map_destroy(handle); }
};

[[nodiscard]] auto fromFfi(std::int32_t code, const AdVizError &error) -> Status {
  if (code == AD_VIZ_OK) {
    return {};
  }
  const auto kind = [code] {
    switch (code) {
    case AD_VIZ_INVALID_INPUT:
      return ErrorCode::InvalidInput;
    case AD_VIZ_EMPTY_COLLECTION:
      return ErrorCode::EmptyCollection;
    case AD_VIZ_SIZE_MISMATCH:
      return ErrorCode::SizeMismatch;
    default:
      return ErrorCode::BackendFailure;
    }
  }();
  return tl::make_unexpected(Error{.code = kind,
                                   .message = error.message[0] != '\0'
                                                  ? std::string{error.message}
                                                  : "Rust visualization backend failed."});
}

[[nodiscard]] auto checkAbi() -> Status {
  if (ad_viz_abi_version() != AD_VIZ_ABI_VERSION) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::BackendFailure, .message = "Visualizer ABI version mismatch."});
  }
  return {};
}

[[nodiscard]] auto toGeometry(const Visualizer::MapGeometry &map) -> Result<AdVizGeometry> {
  if (map.width <= 0 || map.height <= 0 || !std::isfinite(map.resolution) ||
      map.resolution <= 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Map geometry must be positive and finite."});
  }
  return AdVizGeometry{static_cast<std::uint32_t>(map.width),
                       static_cast<std::uint32_t>(map.height), map.resolution};
}

[[nodiscard]] auto toPoints(std::span<const types::Point> points) -> std::vector<AdVizPoint> {
  auto converted = std::vector<AdVizPoint>{};
  converted.reserve(points.size());
  for (const auto &point : points) {
    converted.push_back({point.x, point.y});
  }
  return converted;
}

[[nodiscard]] auto bytes(std::string_view value) -> const std::uint8_t * {
  return reinterpret_cast<const std::uint8_t *>(value.data());
}

} // namespace

struct Visualizer::Backend {
  std::unique_ptr<AdVisualizer, VisualizerDeleter> handle;
};
struct Visualizer::MapStorage {
  std::unique_ptr<AdPreparedMap, MapDeleter> handle;
};

Visualizer::Visualizer() = default;
Visualizer::~Visualizer() = default;
Visualizer::Visualizer(Visualizer &&) noexcept = default;
auto Visualizer::operator=(Visualizer &&) noexcept -> Visualizer & = default;

auto Visualizer::ensureBackend() const -> Status {
  if (backend_) {
    return {};
  }
  if (const auto status = checkAbi(); !status) {
    return status;
  }
  auto backend = std::make_unique<Backend>();
  AdVisualizer *raw = nullptr;
  AdVizError error{};
  const auto code = ad_viz_new(&raw, &error);
  backend->handle.reset(raw);
  if (const auto status = fromFfi(code, error); !status) {
    return status;
  }
  backend_ = std::move(backend);
  return {};
}

auto Visualizer::prepareMap(const types::MapData &map) -> Result<PreparedMap> {
  if (const auto status = checkAbi(); !status) {
    return tl::make_unexpected(status.error());
  }
  const auto geometry = toGeometry({map.width, map.height, map.resolution});
  if (!geometry) {
    return tl::make_unexpected(geometry.error());
  }
  const auto expected =
      static_cast<std::uint64_t>(map.width) * static_cast<std::uint64_t>(map.height);
  if (map.grid.size() != expected) {
    return tl::make_unexpected(Error{.code = ErrorCode::SizeMismatch,
                                     .message = "Map grid size does not match width and height."});
  }
  auto storage = std::make_shared<MapStorage>();
  AdPreparedMap *raw = nullptr;
  AdVizError error{};
  const auto code = ad_viz_prepare_map({*geometry, map.grid.data(), map.grid.size()}, &raw, &error);
  storage->handle.reset(raw);
  if (const auto status = fromFfi(code, error); !status) {
    return tl::make_unexpected(status.error());
  }
  return PreparedMap{{map.width, map.height, map.resolution}, std::move(storage)};
}

auto Visualizer::renderFrame(const PreparedMap &prepared) const -> Status {
  if (!prepared.storage_) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Prepared map has been moved from."});
  }
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  AdVizError error{};
  const auto code =
      ad_viz_begin_frame(backend_->handle.get(), prepared.storage_->handle.get(), &error);
  return fromFfi(code, error);
}

auto Visualizer::renderPath(std::span<const types::Point> path, const MapGeometry &map) const
    -> Status {
  return renderPath(path, map, "b-");
}

auto Visualizer::renderPath(std::span<const types::Point> path, const MapGeometry &map,
                            std::string_view style) const -> Status {
  if (path.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Path is empty."});
  }
  const auto geometry = toGeometry(map);
  if (!geometry) {
    return tl::make_unexpected(geometry.error());
  }
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  const auto points = toPoints(path);
  AdVizError error{};
  const auto code = ad_viz_render_path(backend_->handle.get(), points.data(), points.size(),
                                       *geometry, bytes(style), style.size(), &error);
  return fromFfi(code, error);
}

auto Visualizer::renderRobot(const types::Pose &pose, const types::Footprint &footprint,
                             const MapGeometry &map) const -> Status {
  if (footprint.vertices.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Footprint has no vertices."});
  }
  const auto geometry = toGeometry(map);
  if (!geometry) {
    return tl::make_unexpected(geometry.error());
  }
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  const auto vertices = toPoints(footprint.vertices);
  AdVizError error{};
  const auto code = ad_viz_render_robot(backend_->handle.get(), {pose.x, pose.y, pose.theta},
                                        vertices.data(), vertices.size(), *geometry, &error);
  return fromFfi(code, error);
}

auto Visualizer::renderMarker(const types::Point &point, const MapGeometry &map, double size,
                              std::string_view color) const -> Status {
  const auto geometry = toGeometry(map);
  if (!geometry) {
    return tl::make_unexpected(geometry.error());
  }
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  AdVizError error{};
  const auto code = ad_viz_render_marker(backend_->handle.get(), {point.x, point.y}, *geometry,
                                         size, bytes(color), color.size(), &error);
  return fromFfi(code, error);
}

auto Visualizer::renderPoints(std::span<const types::Point> points, const MapGeometry &map,
                              double size, std::string_view color) const -> Status {
  if (points.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Point set is empty."});
  }
  const auto geometry = toGeometry(map);
  if (!geometry) {
    return tl::make_unexpected(geometry.error());
  }
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  const auto converted = toPoints(points);
  AdVizError error{};
  const auto code = ad_viz_render_points(backend_->handle.get(), converted.data(), converted.size(),
                                         *geometry, size, bytes(color), color.size(), &error);
  return fromFfi(code, error);
}

auto Visualizer::renderScan(const types::Pose &pose, const types::LidarScan &scan,
                            const MapGeometry &map) const -> Status {
  if (scan.ranges.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Scan is empty."});
  }
  const auto geometry = toGeometry(map);
  if (!geometry) {
    return tl::make_unexpected(geometry.error());
  }
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  const AdVizScan input{scan.ranges.data(), scan.ranges.size(), scan.minAngle, scan.angleIncrement,
                        scan.maxRange};
  AdVizError error{};
  const auto code = ad_viz_render_scan(backend_->handle.get(), {pose.x, pose.y, pose.theta}, input,
                                       *geometry, &error);
  return fromFfi(code, error);
}

auto Visualizer::presentFrame() const -> Status {
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  AdVizError error{};
  const auto code = ad_viz_present_frame(backend_->handle.get(), &error);
  return fromFfi(code, error);
}

auto Visualizer::saveFigure(std::string_view path) const -> Status {
  if (path.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Output path is empty."});
  }
  if (const auto status = ensureBackend(); !status) {
    return status;
  }
  AdVizError error{};
  const auto code = ad_viz_save_figure(backend_->handle.get(), bytes(path), path.size(), &error);
  return fromFfi(code, error);
}

} // namespace ad::visualization
