#include "features/control/pure_pursuit.hpp"
#include "features/planning/astar_planner.hpp"
#include "features/planning/grid_collision_checker.hpp"
#include "features/simulation/collision_checker.hpp"
#include "features/simulation/lidar_sim.hpp"
#include "features/simulation/unicycle_model.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <optional>
#include <ranges>
#include <span>
#include <thread>

namespace ad::demo {

[[nodiscard]] auto makeFootprint() -> types::Footprint {
  return types::Footprint{{{-0.2, -0.1}, {0.3, -0.1}, {0.3, 0.1}, {-0.2, 0.1}}};
}

} // namespace ad::demo

auto main() -> int {
  const auto mapResult = ad::loadMapFromYaml("tools/map.yaml");
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }
  const auto map = *mapResult;
  const ad::types::Pose start{1.0, 1.0, 0.0};
  const ad::types::Pose goal{9.0, 1.0, 0.0};
  const auto footprint = ad::demo::makeFootprint();

  const auto checkerResult = ad::planning::GridCollisionChecker::create(map, footprint);
  if (!checkerResult) {
    fmt::print(stderr, "Collision checker error: {}\n", checkerResult.error().message);
    return 1;
  }

  const ad::planning::AStarPlanner planner{*checkerResult};
  const auto pathResult = planner.plan(map, start, goal, footprint);
  if (!pathResult) {
    fmt::print(stderr, "Planning error: {}\n", pathResult.error().message);
    return 1;
  }

  const ad::visualization::Visualizer viz;
  const auto preparedMapResult = ad::visualization::Visualizer::prepareMap(map);
  if (!preparedMapResult) {
    fmt::print(stderr, "Render error: {}\n", preparedMapResult.error().message);
    return 1;
  }
  const auto &preparedMap = *preparedMapResult;
  const auto &mapGeometry = preparedMap.geometry;

  const ad::control::PurePursuitConfig controllerConfig{.lookaheadDistance = 0.6,
                                                        .desiredLinearVelocity = 1.2};
  const ad::control::PurePursuitController controller{controllerConfig};
  const ad::simulation::UnicycleModel model;
  const ad::simulation::LidarSim lidar;
  const ad::simulation::CollisionCheckConfig collisionConfig{.maxTranslationStep = 0.05,
                                                             .maxRotationStep = 0.05};
  const ad::simulation::CollisionChecker collisionChecker{map, footprint, collisionConfig};

  auto state =
      std::optional<ad::simulation::MotionState>{ad::simulation::MotionState{start, {0.0, 0.0}}};
  constexpr auto dt = 0.2;
  constexpr auto kGoalTolerance = 0.3;
  constexpr auto kFrameDelay = std::chrono::milliseconds{80};
  constexpr int kMaxSteps = 250;

  auto failure = std::optional<ad::Error>{};

  const auto reachedGoal = std::ranges::any_of(std::views::iota(0, kMaxSteps), [&](int step) {
    if (failure) {
      return true;
    }

    const auto input = ad::control::ControlInput{std::span{*pathResult}, state->pose};
    const auto commandResult = controller.computeCommand(input);
    if (!commandResult) {
      failure.emplace(commandResult.error());
      return true;
    }

    const auto scanResult = lidar.simulate(map, state->pose);
    if (!scanResult) {
      failure.emplace(scanResult.error());
      return true;
    }

    const auto frameStatus = viz.renderFrame(preparedMap);
    if (!frameStatus) {
      failure.emplace(frameStatus.error());
      return true;
    }

    const auto pathStatus = viz.renderPath(std::span{*pathResult}, mapGeometry);
    if (!pathStatus) {
      failure.emplace(pathStatus.error());
      return true;
    }

    const auto scanStatus = viz.renderScan(state->pose, *scanResult, mapGeometry);
    if (!scanStatus) {
      failure.emplace(scanStatus.error());
      return true;
    }

    const auto robotStatus = viz.renderRobot(state->pose, footprint, mapGeometry);
    if (!robotStatus) {
      failure.emplace(robotStatus.error());
      return true;
    }

    const auto presentStatus = viz.presentFrame();
    if (!presentStatus) {
      failure.emplace(presentStatus.error());
      return true;
    }

    const auto nextState = model.propagate(*state, *commandResult, dt);
    if (!nextState) {
      failure.emplace(nextState.error());
      return true;
    }

    const auto trajectoryFree = collisionChecker.checkTrajectory(state->pose, nextState->pose);
    if (!trajectoryFree) {
      failure.emplace(trajectoryFree.error());
      return true;
    }
    if (!*trajectoryFree) {
      failure.emplace(
          ad::Error{ad::ErrorCode::InvalidInput, "Collision detected during propagation."});
      return true;
    }

    state.emplace(ad::simulation::MotionState{nextState->pose, nextState->twist});

    const auto distanceToGoal = std::hypot(goal.x - state->pose.x, goal.y - state->pose.y);
    fmt::print("Step {:03d}: distance to goal = {:.3f}\n", step, distanceToGoal);

    std::this_thread::sleep_for(kFrameDelay);
    return distanceToGoal <= kGoalTolerance;
  });

  if (failure) {
    fmt::print(stderr, "Simulation error: {}\n", failure->message);
    return 1;
  }

  if (!reachedGoal) {
    fmt::print(stderr, "Simulation ended before reaching the goal.\n");
    return 1;
  }

  const auto saveStatus = viz.saveFigure("planning_control_lidar_path.png");
  if (!saveStatus) {
    fmt::print(stderr, "Render error: {}\n", saveStatus.error().message);
    return 1;
  }

  fmt::print("Reached goal.\n");
  return 0;
}
