#include "features/control/pure_pursuit.hpp"
#include "features/localization/ekf_localizer.hpp"
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
#include <filesystem>
#include <fmt/core.h>
#include <fstream>
#include <iomanip>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace ad::demo {

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

[[nodiscard]] auto serializePoints(std::span<const types::Point> points) -> std::string {
  auto output = std::string{};
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (index != 0) {
      output.push_back(';');
    }
    output += fmt::format("{:.8f}:{:.8f}", points[index].x, points[index].y);
  }
  return output;
}

[[nodiscard]] auto serializeRanges(std::span<const double> ranges) -> std::string {
  auto output = std::string{};
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    if (index != 0) {
      output.push_back(';');
    }
    output += fmt::format("{:.8f}", ranges[index]);
  }
  return output;
}

[[nodiscard]] auto scanToPoints(const types::Pose &pose, const types::LidarScan &scan)
    -> std::vector<types::Point> {
  auto points = std::vector<types::Point>{};
  points.reserve(scan.ranges.size());
  for (const auto angleIndex : std::views::iota(std::size_t{0}, scan.ranges.size())) {
    const auto angle =
        pose.theta + scan.minAngle + (scan.angleIncrement * static_cast<double>(angleIndex));
    const auto distance = scan.ranges[angleIndex];
    points.push_back(
        types::Point{pose.x + (std::cos(angle) * distance), pose.y + (std::sin(angle) * distance)});
  }
  return points;
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

  auto localizerResult = ad::localization::PureEkfLocalizer::create(
      map, ad::localization::PureEkfLocalizer::defaultConfig());
  if (!localizerResult) {
    fmt::print(stderr, "Localizer error: {}\n", localizerResult.error().message);
    return 1;
  }
  auto localizer = std::move(*localizerResult);
  const auto initialCovariance = std::array<double, 9>{0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.2};
  const auto initStatus = localizer->reset(start, initialCovariance);
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
  constexpr double kScoreThreshold = 0.7;
  constexpr double kMinSpeedScale = 0.4;
  constexpr double kMaxAbsAngular = 2.5;

  std::error_code fsError;
  std::filesystem::create_directories("logs", fsError);
  if (fsError) {
    fmt::print(stderr, "Log directory error: {}\n", fsError.message());
    return 1;
  }
  std::ofstream logFile("logs/localization_control_lidar_demo.log");
  if (!logFile.is_open()) {
    fmt::print(stderr, "Log file error: failed to open log file.\n");
    return 1;
  }
  logFile << std::fixed << std::setprecision(8);
  logFile << "# localization_control_lidar_demo log\n";
  logFile << "# dt=" << dt << ", goal_tolerance=" << kGoalTolerance << ", max_steps=" << kMaxSteps
          << "\n";
  logFile << "# footprint=" << ad::demo::serializePoints(footprint.vertices) << "\n";
  logFile << "# path=" << ad::demo::serializePoints(std::span{*pathResult}) << "\n";
  logFile << "# columns: "
             "step,dist_before,dist_after,pos_err,head_err,score,true_x,true_y,true_theta,est_x,"
             "est_y,est_theta,v,w,scan_points\n";
  logFile << "step,dist_before,dist_after,pos_err,head_err,score,true_x,true_y,true_theta,est_x,"
             "est_y,est_theta,v,w,scan_points\n";

  auto failure = std::optional<ad::Error>{};

  {
    const auto frameStatus = viz.renderFrame(preparedMap);
    if (!frameStatus) {
      fmt::print(stderr, "Render error: {}\n", frameStatus.error().message);
      return 1;
    }

    const auto pathStatus = viz.renderPath(std::span{*pathResult}, mapGeometry);
    if (!pathStatus) {
      fmt::print(stderr, "Render error: {}\n", pathStatus.error().message);
      return 1;
    }

    const auto robotStatus = viz.renderRobot(trueState->pose, footprint, mapGeometry);
    if (!robotStatus) {
      fmt::print(stderr, "Render error: {}\n", robotStatus.error().message);
      return 1;
    }

    const auto presentStatus = viz.presentFrame();
    if (!presentStatus) {
      fmt::print(stderr, "Render error: {}\n", presentStatus.error().message);
      return 1;
    }
  }

  const auto reachedGoal = std::ranges::any_of(std::views::iota(0, kMaxSteps), [&](int step) {
    if (failure) {
      return true;
    }

    const auto estimateResult = localizer->estimate();
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

    const auto scoreScale = estimateResult->score < kScoreThreshold
                                ? std::max(kMinSpeedScale, estimateResult->score / kScoreThreshold)
                                : 1.0;
    const auto scaledV = commandResult->v * scoreScale;
    const auto scaledW = commandResult->w;
    const auto appliedCommand =
        ad::types::Twist{.v = scaledV, .w = std::clamp(scaledW, -kMaxAbsAngular, kMaxAbsAngular)};

    const auto predictStatus = localizer->predict(appliedCommand, dt);
    if (!predictStatus) {
      failure.emplace(predictStatus.error());
      return true;
    }

    const auto scanResult = lidar.simulate(map, trueState->pose);
    if (!scanResult) {
      failure.emplace(scanResult.error());
      return true;
    }
    const auto scanPoints = ad::demo::scanToPoints(trueState->pose, *scanResult);

    const auto updateStatus = localizer->update(*scanResult, map);
    if (!updateStatus) {
      failure.emplace(updateStatus.error());
      return true;
    }

    const auto updatedEstimate = localizer->estimate();
    if (!updatedEstimate) {
      failure.emplace(updatedEstimate.error());
      return true;
    }

    const auto dx = updatedEstimate->pose.x - trueState->pose.x;
    const auto dy = updatedEstimate->pose.y - trueState->pose.y;
    const auto positionError = std::hypot(dx, dy);
    const auto headingError =
        std::abs(ad::demo::normalizeAngle(updatedEstimate->pose.theta - trueState->pose.theta));
    const auto distanceBefore = std::hypot(goal.x - trueState->pose.x, goal.y - trueState->pose.y);

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

    const auto nextState = model.propagate(*trueState, appliedCommand, dt);
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

    const auto distanceAfter = std::hypot(goal.x - trueState->pose.x, goal.y - trueState->pose.y);

    logFile << step << ',' << distanceBefore << ',' << distanceAfter << ',' << positionError << ','
            << headingError << ',' << updatedEstimate->score << ',' << trueState->pose.x << ','
            << trueState->pose.y << ',' << trueState->pose.theta << ',' << updatedEstimate->pose.x
            << ',' << updatedEstimate->pose.y << ',' << updatedEstimate->pose.theta << ','
            << appliedCommand.v << ',' << appliedCommand.w << ','
            << ad::demo::serializePoints(scanPoints) << '\n';

    std::this_thread::sleep_for(kFrameDelay);
    return distanceAfter <= kGoalTolerance;
  });

  if (failure) {
    fmt::print(stderr, "Simulation error: {}\n", failure->message);
  }

  if (!reachedGoal) {
    fmt::print(stderr, "Simulation ended before reaching the goal.\n");
  }

  logFile << "# result=" << (reachedGoal && !failure ? "success" : "failure") << '\n';
  if (failure) {
    logFile << "# error=" << failure->message << '\n';
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
