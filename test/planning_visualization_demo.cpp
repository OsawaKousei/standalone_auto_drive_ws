#include "features/planning/dijkstra_planner.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <cstdint>
#include <fmt/core.h>
#include <span>
#include <vector>

namespace ad::demo {

[[nodiscard]] auto makeMap() -> types::MapData {
  const int width = 6;
  const int height = 4;
  const double resolution = 0.5;
  auto grid = std::vector<std::int8_t>(static_cast<std::size_t>(width * height), 0);

  grid[static_cast<std::size_t>(1 * width + 3)] = 1;
  grid[static_cast<std::size_t>(2 * width + 4)] = 1;
  grid[static_cast<std::size_t>(0 * width + 1)] = 1;

  return types::MapData{width, height, resolution, grid};
}

} // namespace ad::demo

auto main() -> int {
  const auto map = ad::demo::makeMap();
  const ad::types::Pose start{0.25, 0.25, 0.0};
  const ad::types::Pose goal{2.75, 1.25, 0.0};

  const ad::planning::DijkstraPlanner planner;
  const auto pathResult = planner.plan(map, start, goal);
  if (!pathResult) {
    fmt::print(stderr, "Planning error: {}\n", pathResult.error().message);
    return 1;
  }

  fmt::print("Preparing map...\n");
  const ad::visualization::Visualizer viz;
  const auto preparedMapResult = ad::visualization::Visualizer::prepareMap(map);
  if (!preparedMapResult) {
    fmt::print(stderr, "Render error: {}\n", preparedMapResult.error().message);
    return 1;
  }

  fmt::print("Rendering frame...\n");
  const auto frameStatus = viz.renderFrame(*preparedMapResult);
  if (!frameStatus) {
    fmt::print(stderr, "Render error: {}\n", frameStatus.error().message);
    return 1;
  }

  fmt::print("Rendering path...\n");
  const auto pathStatus = viz.renderPath(std::span{*pathResult}, preparedMapResult->geometry);
  if (!pathStatus) {
    fmt::print(stderr, "Render error: {}\n", pathStatus.error().message);
    return 1;
  }

  fmt::print("Saving image...\n");
  const auto saveStatus = viz.saveFigure("planning_path.png");
  if (!saveStatus) {
    fmt::print(stderr, "Render error: {}\n", saveStatus.error().message);
    return 1;
  }

  return 0;
}
