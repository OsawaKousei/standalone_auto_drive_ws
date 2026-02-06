#include "features/simulation/lidar_sim.hpp"
#include "features/simulation/unicycle_model.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <cstdint>
#include <fmt/core.h>
#include <optional>
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

[[nodiscard]] auto makePath() -> types::Path {
  return types::Path{{0.0, 0.0}, {1.0, 0.5}, {2.0, 0.75}, {3.0, 1.0}};
}

} // namespace ad::demo

auto main() -> int {
  const auto map = ad::demo::makeMap();
  const auto path = ad::demo::makePath();
  const ad::simulation::UnicycleModel model;
  const ad::simulation::LidarSim lidar;
  const ad::visualization::Visualizer viz;

  auto state = std::optional<ad::simulation::MotionState>{
      ad::simulation::MotionState{{0.0, 0.0, 0.0}, {0.0, 0.0}}};
  constexpr auto dt = 0.5;
  const ad::types::Twist command{1.2, 0.6};

  for (int step = 0; step < 8; ++step) {
    fmt::print("\n=== Step {} ===\n", step);

    const auto &current = *state;

    const auto scanResult = lidar.simulate(map, current.pose);
    if (!scanResult) {
      fmt::print(stderr, "Lidar error: {}\n", scanResult.error().message);
      return 1;
    }

    const auto renderStatus = viz.renderFrame(map, current.pose, std::span{path}, *scanResult);
    if (!renderStatus) {
      fmt::print(stderr, "Render error: {}\n", renderStatus.error().message);
      return 1;
    }

    const auto nextState = model.propagate(current, command, dt);
    if (!nextState) {
      fmt::print(stderr, "Propagate error: {}\n", nextState.error().message);
      return 1;
    }

    state.emplace(ad::simulation::MotionState{nextState->pose, nextState->twist});
  }

  fmt::print("\nSimulation finished.\n");
  return 0;
}
