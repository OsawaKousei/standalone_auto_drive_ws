#include "features/planning/astar_planner.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/types.hpp"

#include <fmt/core.h>
#include <span>

auto main() -> int {
  const auto mapResult = ad::loadMapFromYaml("tools/map.yaml");
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }
  const auto map = *mapResult;
  const ad::types::Pose start{0.6, 0.6, 0.0};
  const ad::types::Pose goal{8.4, 5.4, 0.0};

  const ad::planning::AStarPlanner planner;
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
