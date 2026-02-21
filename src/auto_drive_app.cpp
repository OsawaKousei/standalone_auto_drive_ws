#include "features/simulation/collision_checker.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/scenario_runtime.hpp"
#include "shared/types.hpp"

#include <algorithm>
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

[[nodiscard]] auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
  if (angle < 0.0) {
    angle += 2.0 * std::numbers::pi;
  }
  return angle - std::numbers::pi;
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

[[nodiscard]] auto scanToPoints(const types::Pose &pose, const types::LidarScan &scan)
    -> std::vector<types::Point> {
  auto points = std::vector<types::Point>{};
  points.reserve(scan.ranges.size());
  for (const auto angleIndex : std::views::iota(std::size_t{0}, scan.ranges.size())) {
    const auto angle =
        pose.theta + scan.minAngle + (scan.angleIncrement * static_cast<double>(angleIndex));
    const auto distance = scan.ranges[angleIndex];
    points.push_back(types::Point{.x = pose.x + (distance * std::cos(angle)),
                                  .y = pose.y + (distance * std::sin(angle))});
  }
  return points;
}

} // namespace ad::demo

auto main(int argc, char **argv) -> int {
  const auto scenarioPath = std::string{argc > 1 ? argv[1] : "configs/scenario.toml"};
  const auto scenarioResult = ad::scenario::loadScenario(scenarioPath);
  if (!scenarioResult) {
    fmt::print(stderr, "Scenario load error: {}\n", scenarioResult.error().message);
    return 1;
  }
  const auto &scenario = *scenarioResult;

  const auto mapPath = ad::scenario::resolvePath(scenario, scenario.mapYamlPath);
  const auto mapResult = ad::loadMapFromYaml(mapPath);
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }
  const auto map = *mapResult;
  const auto start = scenario.start;
  const auto goal = scenario.goal;
  const auto &footprint = scenario.footprint;

  auto plannerResult = ad::scenario::createPlanner(scenario, map, footprint);
  if (!plannerResult) {
    fmt::print(stderr, "Planner create error: {}\n", plannerResult.error().message);
    return 1;
  }
  auto planner = std::move(*plannerResult);
  const auto pathResult = planner->plan(map, start, goal, footprint);
  if (!pathResult) {
    fmt::print(stderr, "Planning error: {}\n", pathResult.error().message);
    return 1;
  }

  auto localizerResult = ad::scenario::createLocalizer(scenario, map);
  if (!localizerResult) {
    fmt::print(stderr, "Localizer error: {}\n", localizerResult.error().message);
    return 1;
  }
  auto localizer = std::move(*localizerResult);
  const auto initStatus = localizer->reset(start, scenario.initialCovariance);
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

  auto controllerResult = ad::scenario::createController(scenario);
  if (!controllerResult) {
    fmt::print(stderr, "Controller create error: {}\n", controllerResult.error().message);
    return 1;
  }
  auto controller = std::move(*controllerResult);

  auto sensorResult = ad::scenario::createSensor(scenario);
  if (!sensorResult) {
    fmt::print(stderr, "Sensor create error: {}\n", sensorResult.error().message);
    return 1;
  }
  auto sensor = std::move(*sensorResult);

  auto physicsResult = ad::scenario::createPhysics(scenario);
  if (!physicsResult) {
    fmt::print(stderr, "Physics create error: {}\n", physicsResult.error().message);
    return 1;
  }
  auto physics = std::move(*physicsResult);

  const ad::simulation::CollisionChecker collisionChecker{map, footprint, scenario.collision};

  auto trueState = std::optional<ad::simulation::MotionState>{
      ad::simulation::MotionState{.pose = start, .twist = ad::types::Twist{.v = 0.0, .w = 0.0}}};
  const auto deltaT = scenario.runtime.deltaT;
  const auto kGoalTolerance = scenario.runtime.goalTolerance;
  const auto kFrameDelay = std::chrono::milliseconds{scenario.runtime.frameDelayMs};
  const auto kMaxSteps = scenario.runtime.maxSteps;
  const auto kScoreThreshold = scenario.runtime.scoreThreshold;
  const auto kMinSpeedScale = scenario.runtime.minSpeedScale;
  const auto kMaxAbsAngular = scenario.runtime.maxAbsAngular;

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
  logFile << "# dt=" << deltaT << ", goal_tolerance=" << kGoalTolerance
          << ", max_steps=" << kMaxSteps << "\n";
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

  const auto reachedGoal =
      std::ranges::any_of(std::views::iota(0, kMaxSteps), [&](int step) -> bool {
        if (failure) {
          return true;
        }

        const auto estimateResult = localizer->estimate();
        if (!estimateResult) {
          failure.emplace(estimateResult.error());
          return true;
        }

        const auto input = ad::control::ControlInput{.path = std::span{*pathResult},
                                                     .currentPose = estimateResult->pose};
        const auto commandResult = controller->computeCommand(input);
        if (!commandResult) {
          failure.emplace(commandResult.error());
          return true;
        }

        const auto scoreScale =
            estimateResult->score < kScoreThreshold
                ? std::max(kMinSpeedScale, estimateResult->score / kScoreThreshold)
                : 1.0;
        const auto scaledV = commandResult->v * scoreScale;
        const auto scaledW = commandResult->w;
        const auto appliedCommand = ad::types::Twist{
            .v = scaledV, .w = std::clamp(scaledW, -kMaxAbsAngular, kMaxAbsAngular)};

        const auto predictStatus = localizer->predict(appliedCommand, deltaT);
        if (!predictStatus) {
          failure.emplace(predictStatus.error());
          return true;
        }

        const auto scanResult = sensor->simulate(map, trueState->pose);
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

        const auto deltaX = updatedEstimate->pose.x - trueState->pose.x;
        const auto deltaY = updatedEstimate->pose.y - trueState->pose.y;
        const auto positionError = std::hypot(deltaX, deltaY);
        const auto headingError =
            std::abs(ad::demo::normalizeAngle(updatedEstimate->pose.theta - trueState->pose.theta));
        const auto distanceBefore =
            std::hypot(goal.x - trueState->pose.x, goal.y - trueState->pose.y);

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

        const auto nextState = physics->propagate(*trueState, appliedCommand, deltaT);
        if (!nextState) {
          failure.emplace(nextState.error());
          return true;
        }

        const auto trajectoryFree =
            collisionChecker.checkTrajectory(trueState->pose, nextState->pose);
        if (!trajectoryFree) {
          failure.emplace(trajectoryFree.error());
          return true;
        }
        if (!*trajectoryFree) {
          failure.emplace(ad::Error{.code = ad::ErrorCode::InvalidInput,
                                    .message = "Collision detected in trajectory propagation."});
          return true;
        }

        trueState.emplace(
            ad::simulation::MotionState{.pose = nextState->pose, .twist = nextState->twist});

        const auto distanceAfter =
            std::hypot(goal.x - trueState->pose.x, goal.y - trueState->pose.y);

        logFile << step << ',' << distanceBefore << ',' << distanceAfter << ',' << positionError
                << ',' << headingError << ',' << updatedEstimate->score << ',' << trueState->pose.x
                << ',' << trueState->pose.y << ',' << trueState->pose.theta << ','
                << updatedEstimate->pose.x << ',' << updatedEstimate->pose.y << ','
                << updatedEstimate->pose.theta << ',' << appliedCommand.v << ',' << appliedCommand.w
                << ',' << ad::demo::serializePoints(scanPoints) << '\n';

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
