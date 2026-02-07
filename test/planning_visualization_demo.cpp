#include "features/planning/astar_planner.hpp"
#include "features/planning/dijkstra_planner.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <cstdint>
#include <fmt/core.h>
#include <ranges>
#include <span>
#include <vector>

namespace ad::demo {

[[nodiscard]] auto makeMap() -> types::MapData {
  const int width = 50;
  const int height = 30;
  const double resolution = 0.2;
  auto grid = std::vector<std::int8_t>(static_cast<std::size_t>(width * height), 0);

  const auto setCell = [&](int x, int y, std::int8_t value) {
    const auto index = static_cast<std::size_t>((y * width) + x);
    grid[index] = value;
  };

  const auto fillHorizontal = [&](int y, int xStart, int xEnd) {
    for (const auto x : std::views::iota(xStart, xEnd + 1)) {
      setCell(x, y, 1);
    }
  };

  const auto fillVertical = [&](int x, int yStart, int yEnd) {
    for (const auto y : std::views::iota(yStart, yEnd + 1)) {
      setCell(x, y, 1);
    }
  };

  fillHorizontal(0, 0, width - 1);
  fillHorizontal(height - 1, 0, width - 1);
  fillVertical(0, 0, height - 1);
  fillVertical(width - 1, 0, height - 1);

  fillHorizontal(10, 2, 46);
  fillVertical(15, 2, 27);
  fillHorizontal(20, 3, 45);
  fillVertical(32, 5, 26);

  fillVertical(9, 1, 9);
  fillVertical(9, 12, 18);
  fillVertical(9, 22, height - 2);

  fillHorizontal(5, 1, 8);
  fillHorizontal(5, 12, 31);
  fillHorizontal(5, 34, width - 2);

  fillHorizontal(25, 1, 31);
  fillHorizontal(25, 34, width - 2);

  const auto openCell = [&](int x, int y) { setCell(x, y, 0); };
  openCell(5, 5);
  openCell(10, 10);
  openCell(16, 5);
  openCell(20, 20);
  openCell(15, 12);
  openCell(32, 22);
  openCell(40, 25);

  return types::MapData{width, height, resolution, grid};
}

} // namespace ad::demo

auto main() -> int {
  const auto map = ad::demo::makeMap();
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
