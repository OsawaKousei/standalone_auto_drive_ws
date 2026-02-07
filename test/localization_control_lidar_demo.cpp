#include "features/control/pure_pursuit.hpp"
#include "features/localization/pure_ekf_localizer.hpp"
#include "features/planning/astar_planner.hpp"
#include "features/planning/grid_collision_checker.hpp"
#include "features/simulation/collision_checker.hpp"
#include "features/simulation/lidar_sim.hpp"
#include "features/simulation/unicycle_model.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <optional>
#include <ranges>
#include <span>
#include <thread>
#include <vector>

namespace ad::demo {

struct ErrorSample {
  int step;
  double position;
  double heading;
};

[[nodiscard]] auto makeFootprint() -> types::Footprint {
  return types::Footprint{{{-0.2, -0.1}, {0.3, -0.1}, {0.3, 0.1}, {-0.2, 0.1}}};
}

[[nodiscard]] auto normalizeAngle(double angle) -> double {
  constexpr double kPi = 3.14159265358979323846;
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
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

  const auto localizerResult = ad::localization::PureEkfLocalizer::create(
      map, ad::localization::PureEkfLocalizer::defaultConfig());
  if (!localizerResult) {
    fmt::print(stderr, "Localizer error: {}\n", localizerResult.error().message);
    return 1;
  }
  auto localizer = std::move(*localizerResult);
  const auto initialCovariance = std::array<double, 9>{0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.2};
  const auto initStatus = localizer.reset(start, initialCovariance);
  if (!initStatus) {
    fmt::print(stderr, "Localizer error: {}\n", initStatus.error().message);
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

  auto trueState =
      std::optional<ad::simulation::MotionState>{ad::simulation::MotionState{start, {0.0, 0.0}}};
  constexpr auto dt = 0.2;
  constexpr auto kGoalTolerance = 0.3;
  constexpr auto kFrameDelay = std::chrono::milliseconds{80};
  constexpr int kMaxSteps = 250;

  auto failure = std::optional<ad::Error>{};
  auto errorHistory = std::vector<ad::demo::ErrorSample>{};
  errorHistory.reserve(kMaxSteps);

  const auto reachedGoal = std::ranges::any_of(std::views::iota(0, kMaxSteps), [&](int step) {
    if (failure) {
      return true;
    }

    const auto estimateResult = localizer.estimate();
    if (!estimateResult) {
      failure.emplace(estimateResult.error());
      return true;
    }

    const auto input = ad::control::ControlInput{std::span{*pathResult}, estimateResult->pose};
    const auto commandResult = controller.computeCommand(input);
    if (!commandResult) {
      failure.emplace(commandResult.error());
      return true;
    }

    const auto predictStatus = localizer.predict(*commandResult, dt);
    if (!predictStatus) {
      failure.emplace(predictStatus.error());
      return true;
    }

    const auto scanResult = lidar.simulate(map, trueState->pose);
    if (!scanResult) {
      failure.emplace(scanResult.error());
      return true;
    }

    const auto updateStatus = localizer.update(*scanResult, map);
    if (!updateStatus) {
      failure.emplace(updateStatus.error());
      return true;
    }

    const auto updatedEstimate = localizer.estimate();
    if (!updatedEstimate) {
      failure.emplace(updatedEstimate.error());
      return true;
    }

    const auto dx = updatedEstimate->pose.x - trueState->pose.x;
    const auto dy = updatedEstimate->pose.y - trueState->pose.y;
    const auto positionError = std::hypot(dx, dy);
    const auto headingError =
        std::abs(ad::demo::normalizeAngle(updatedEstimate->pose.theta - trueState->pose.theta));
    errorHistory.push_back(ad::demo::ErrorSample{step, positionError, headingError});

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

    const auto scanStatus = viz.renderScan(trueState->pose, *scanResult, mapGeometry);
    if (!scanStatus) {
      failure.emplace(scanStatus.error());
      return true;
    }

    const auto robotStatus = viz.renderRobot(trueState->pose, footprint, mapGeometry);
    if (!robotStatus) {
      failure.emplace(robotStatus.error());
      return true;
    }

    const auto presentStatus = viz.presentFrame();
    if (!presentStatus) {
      failure.emplace(presentStatus.error());
      return true;
    }

    const auto nextState = model.propagate(*trueState, *commandResult, dt);
    if (!nextState) {
      failure.emplace(nextState.error());
      return true;
    }

    const auto trajectoryFree = collisionChecker.checkTrajectory(trueState->pose, nextState->pose);
    if (!trajectoryFree) {
      failure.emplace(trajectoryFree.error());
      return true;
    }
    if (!*trajectoryFree) {
      failure.emplace(
          ad::Error{ad::ErrorCode::InvalidInput, "Collision detected during propagation."});
      return true;
    }

    trueState.emplace(ad::simulation::MotionState{nextState->pose, nextState->twist});

    const auto distanceToGoal = std::hypot(goal.x - trueState->pose.x, goal.y - trueState->pose.y);
    fmt::print("Step {:03d}: distance to goal = {:.3f}\n", step, distanceToGoal);

    std::this_thread::sleep_for(kFrameDelay);
    return distanceToGoal <= kGoalTolerance;
  });

  if (!errorHistory.empty()) {
    fmt::print("Localization error timeline (step, position_m, heading_rad):\n");
    for (const auto &sample : errorHistory) {
      fmt::print("  {:03d}, {:.4f}, {:.4f}\n", sample.step, sample.position, sample.heading);
    }

    double sumPos = 0.0;
    double sumPosSq = 0.0;
    double sumHeading = 0.0;
    double sumHeadingSq = 0.0;
    for (const auto &sample : errorHistory) {
      sumPos += sample.position;
      sumPosSq += sample.position * sample.position;
      sumHeading += sample.heading;
      sumHeadingSq += sample.heading * sample.heading;
    }

    const auto count = static_cast<double>(errorHistory.size());
    const auto meanPos = sumPos / count;
    const auto rmsPos = std::sqrt(sumPosSq / count);
    const auto meanHeading = sumHeading / count;
    const auto rmsHeading = std::sqrt(sumHeadingSq / count);

    fmt::print("Localization accuracy over {} steps:\n", errorHistory.size());
    fmt::print("  Position mean = {:.4f} m, RMS = {:.4f} m\n", meanPos, rmsPos);
    fmt::print("  Heading  mean = {:.4f} rad, RMS = {:.4f} rad\n", meanHeading, rmsHeading);
  }

  if (failure) {
    fmt::print(stderr, "Simulation error: {}\n", failure->message);
  }

  if (!reachedGoal) {
    fmt::print(stderr, "Simulation ended before reaching the goal.\n");
  }

  const auto saveStatus = viz.saveFigure("localization_control_lidar_path.png");
  if (!saveStatus) {
    fmt::print(stderr, "Render error: {}\n", saveStatus.error().message);
    return 1;
  }

  if (reachedGoal && !failure) {
    fmt::print("Reached goal.\n");
    return 0;
  }

  return 1;
}
