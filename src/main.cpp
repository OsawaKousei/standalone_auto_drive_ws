#include "shared/result.hpp"
#include "shared/types.hpp"

#include <cstdint>
#include <fmt/core.h>
#include <span>
#include <vector>

namespace ad {

[[nodiscard]] auto validatePath(std::span<const types::Point> path) -> Status {
  if (path.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Path is empty."});
  }
  return {};
}

[[nodiscard]] auto validateMap(const types::MapData &map) -> Status {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  if (map.grid.size() != expectedCells) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }
  return {};
}

} // namespace ad

auto main() -> int {
  const auto demoMap = [] {
    const int width = 3;
    const int height = 3;
    const double resolution = 0.5;
    const std::vector<std::int8_t> grid(width * height, 0);
    return ad::types::MapData{width, height, resolution, grid};
  }();

  const ad::types::Path path = {{0.0, 0.0}, {0.5, 0.25}, {1.0, 0.5}};

  const auto pathStatus = ad::validatePath(std::span{path});
  if (!pathStatus) {
    fmt::print(stderr, "Path error: {}\n", pathStatus.error().message);
    return 1;
  }

  const auto mapStatus = ad::validateMap(demoMap);
  if (!mapStatus) {
    fmt::print(stderr, "Map error: {}\n", mapStatus.error().message);
    return 1;
  }

  for (const auto &point : path) {
    fmt::print("Waypoint -> x: {:.2f}, y: {:.2f}\n", point.x, point.y);
  }

  fmt::print("Map initialized: {}x{} @ {:.2f} m/cell\n", demoMap.width, demoMap.height,
             demoMap.resolution);
  return 0;
}
