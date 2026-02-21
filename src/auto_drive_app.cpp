#include "features/simulation/collision_checker.hpp"
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "features/visualization/visualizer.hpp"
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/scenario_runtime.hpp"
#include "shared/types.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <fstream>
#include <iomanip>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <vector>

namespace ad::demo {

constexpr auto kAnglePeriod =
    2.0 *
    std::numbers::pi; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
constexpr auto kLogPrecision =
    8; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
constexpr auto kLidarScheduleEpsilon =
    1.0e-12; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

[[nodiscard]] auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, kAnglePeriod);
  if (angle < 0.0) {
    angle += kAnglePeriod;
  }
  return angle - std::numbers::pi;
}

[[nodiscard]] auto serializePoints(std::span<const types::Point> points) -> std::string {
  auto output = std::string{};
  for (const auto index : std::views::iota(std::size_t{0}, points.size())) {
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

[[nodiscard]] auto distanceToGoal(const types::Pose &pose, const types::Pose &goal) -> double {
  return std::hypot(goal.x - pose.x, goal.y - pose.y);
}

[[nodiscard]] auto initializeLogFile(double odometryDeltaT, double lidarDeltaT, double renderDeltaT,
                                     double goalTolerance, int maxSteps,
                                     std::span<const types::Point> footprint,
                                     std::span<const types::Point> path)
    -> std::optional<std::ofstream> {
  std::error_code fsError;
  std::filesystem::create_directories("logs", fsError);
  if (fsError) {
    fmt::print(stderr, "Log directory error: {}\n", fsError.message());
    return std::nullopt;
  }

  auto logFile = std::ofstream{"logs/localization_control_lidar_demo.log"};
  if (!logFile.is_open()) {
    fmt::print(stderr, "Log file error: failed to open log file.\n");
    return std::nullopt;
  }

  logFile << std::fixed << std::setprecision(kLogPrecision);
  logFile << "# localization_control_lidar_demo log\n";
  logFile << "# odometry_dt=" << odometryDeltaT << ", lidar_dt=" << lidarDeltaT
          << ", render_dt=" << renderDeltaT << ", goal_tolerance=" << goalTolerance
          << ", max_steps=" << maxSteps << "\n";
  logFile << "# footprint=" << serializePoints(footprint) << "\n";
  logFile << "# path=" << serializePoints(path) << "\n";
  logFile << "# columns: "
             "step,dist_before,dist_after,pos_err,head_err,score,true_x,true_y,true_theta,est_x,"
             "est_y,est_theta,v,w,lidar_updated,odom_df,odom_dl,odom_dtheta,scan_points\n";
  logFile << "step,dist_before,dist_after,pos_err,head_err,score,true_x,true_y,true_theta,est_x,"
             "est_y,est_theta,v,w,lidar_updated,odom_df,odom_dl,odom_dtheta,scan_points\n";
  return logFile;
}

[[nodiscard]] auto renderFrame(const ad::visualization::Visualizer &viz, const auto &preparedMap,
                               std::span<const types::Point> path,
                               const std::optional<std::vector<types::Point>> &scanWorldPoints,
                               const types::Pose &robotPose, const types::Footprint &footprint)
    -> std::optional<ad::Error> {
  const auto &mapGeometry = preparedMap.geometry;
  const auto frameStatus = viz.renderFrame(preparedMap);
  if (!frameStatus) {
    return frameStatus.error();
  }

  const auto pathStatus = viz.renderPath(path, mapGeometry);
  if (!pathStatus) {
    return pathStatus.error();
  }

  if (scanWorldPoints.has_value() && !scanWorldPoints->empty()) {
    const auto scanStatus = viz.renderPoints(*scanWorldPoints, mapGeometry, 10.0, "green");
    if (!scanStatus) {
      return scanStatus.error();
    }
  }

  const auto robotStatus = viz.renderRobot(robotPose, footprint, mapGeometry);
  if (!robotStatus) {
    return robotStatus.error();
  }

  const auto presentStatus = viz.presentFrame();
  if (!presentStatus) {
    return presentStatus.error();
  }
  return std::nullopt;
}

auto appendLogEntry(std::ofstream &logFile, int step, double distanceBefore, double distanceAfter,
                    double positionError, double headingError, double estimateScore,
                    const types::Pose &truePose, const types::Pose &estimatedPose,
                    const types::Twist &appliedCommand, bool lidarUpdated,
                    const auto &odometryDelta, std::span<const types::Point> scanPoints) -> void {
  logFile << step << ',' << distanceBefore << ',' << distanceAfter << ',' << positionError << ','
          << headingError << ',' << estimateScore << ',' << truePose.x << ',' << truePose.y << ','
          << truePose.theta << ',' << estimatedPose.x << ',' << estimatedPose.y << ','
          << estimatedPose.theta << ',' << appliedCommand.v << ',' << appliedCommand.w << ','
          << (lidarUpdated ? 1 : 0) << ',' << odometryDelta.deltaForward << ','
          << odometryDelta.deltaLateral << ',' << odometryDelta.deltaTheta << ','
          << serializePoints(scanPoints) << '\n';
}

struct SimulationOutcome {
  bool reachedGoal{false};
  std::optional<ad::Error> failure{};
};

[[nodiscard]] auto
runSimulationLoop(const types::Pose &start, const types::Pose &goal,
                  const types::Footprint &footprint, std::span<const types::Point> path,
                  const auto &map, const ad::simulation::CollisionChecker &collisionChecker,
                  double odometryDeltaT, double lidarDeltaT, double renderDeltaT,
                  double goalTolerance, int maxSteps, const ad::visualization::Visualizer &viz,
                  const auto &preparedMap, auto &controller, auto &localizer, auto &lidarSensor,
                  auto &odometrySensor, auto &physics, std::ofstream &logFile)
    -> SimulationOutcome {
  auto outcome = SimulationOutcome{};
  auto trueState = std::optional<ad::simulation::MotionState>{
      ad::simulation::MotionState{.pose = start, .twist = ad::types::Twist{.v = 0.0, .w = 0.0}}};
  auto lidarElapsed = 0.0;
  auto renderElapsed = 0.0;
  auto lastScanWorldPoints = std::optional<std::vector<ad::types::Point>>{};

  for (const auto step : std::views::iota(0, maxSteps)) {
    const auto estimateResult = localizer->estimate();
    if (!estimateResult) {
      outcome.failure.emplace(estimateResult.error());
      break;
    }

    const auto input = ad::control::ControlInput{.path = path, .currentPose = estimateResult->pose};
    const auto commandResult = controller->computeCommand(input);
    if (!commandResult) {
      outcome.failure.emplace(commandResult.error());
      break;
    }
    const auto appliedCommand = *commandResult;

    const auto distanceBefore = distanceToGoal(trueState->pose, goal);

    const auto nextState = physics->propagate(*trueState, appliedCommand, odometryDeltaT);
    if (!nextState) {
      outcome.failure.emplace(nextState.error());
      break;
    }

    const auto trajectoryFree = collisionChecker.checkTrajectory(trueState->pose, nextState->pose);
    if (!trajectoryFree) {
      outcome.failure.emplace(trajectoryFree.error());
      break;
    }
    if (!*trajectoryFree) {
      outcome.failure.emplace(
          ad::Error{.code = ad::ErrorCode::InvalidInput,
                    .message = "Collision detected in trajectory propagation."});
      break;
    }

    const auto odometryDelta = odometrySensor->measure(trueState->pose, nextState->pose);
    if (!odometryDelta) {
      outcome.failure.emplace(odometryDelta.error());
      break;
    }

    const auto predictStatus = localizer->predictOdometry(*odometryDelta);
    if (!predictStatus) {
      outcome.failure.emplace(predictStatus.error());
      break;
    }

    trueState.emplace(
        ad::simulation::MotionState{.pose = nextState->pose, .twist = nextState->twist});

    auto lidarUpdated = false;
    auto scanPoints = std::vector<ad::types::Point>{};
    lidarElapsed += odometryDeltaT;
    if (lidarElapsed + kLidarScheduleEpsilon >= lidarDeltaT) {
      const auto scanResult = lidarSensor->simulate(map, trueState->pose);
      if (!scanResult) {
        outcome.failure.emplace(scanResult.error());
        break;
      }

      const auto updateStatus = localizer->update(*scanResult, map);
      if (!updateStatus) {
        outcome.failure.emplace(updateStatus.error());
        break;
      }

      scanPoints = scanToPoints(trueState->pose, *scanResult);
      lastScanWorldPoints.emplace(scanPoints);
      lidarUpdated = true;
      lidarElapsed = std::fmod(lidarElapsed, lidarDeltaT);
    }

    const auto updatedEstimate = localizer->estimate();
    if (!updatedEstimate) {
      outcome.failure.emplace(updatedEstimate.error());
      break;
    }

    const auto deltaX = updatedEstimate->pose.x - trueState->pose.x;
    const auto deltaY = updatedEstimate->pose.y - trueState->pose.y;
    const auto positionError = std::hypot(deltaX, deltaY);
    const auto headingError =
        std::abs(normalizeAngle(updatedEstimate->pose.theta - trueState->pose.theta));

    renderElapsed += odometryDeltaT;
    const auto shouldRender =
        lidarUpdated || (renderElapsed + kLidarScheduleEpsilon >= renderDeltaT);
    if (shouldRender) {
      const auto renderError =
          renderFrame(viz, preparedMap, path, lastScanWorldPoints, trueState->pose, footprint);
      if (renderError) {
        outcome.failure.emplace(*renderError);
        break;
      }
      renderElapsed = std::fmod(renderElapsed, renderDeltaT);
    }

    const auto distanceAfter = distanceToGoal(trueState->pose, goal);
    appendLogEntry(logFile, step, distanceBefore, distanceAfter, positionError, headingError,
                   updatedEstimate->score, trueState->pose, updatedEstimate->pose, appliedCommand,
                   lidarUpdated, *odometryDelta, scanPoints);

    if (distanceAfter <= goalTolerance) {
      outcome.reachedGoal = true;
      break;
    }
  }

  return outcome;
}

} // namespace ad::demo

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto main(int argc, char **argv) -> int {
  const auto arguments = std::span<char *>{argv, static_cast<std::size_t>(argc)};
  const auto scenarioPath =
      std::string{arguments.size() > std::size_t{1} ? arguments[1] : "configs/scenario.toml"};
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

  auto controllerResult = ad::scenario::createController(scenario);
  if (!controllerResult) {
    fmt::print(stderr, "Controller create error: {}\n", controllerResult.error().message);
    return 1;
  }
  auto controller = std::move(*controllerResult);

  auto lidarSensorResult = ad::scenario::createLidarSensor(scenario);
  if (!lidarSensorResult) {
    fmt::print(stderr, "Lidar sensor create error: {}\n", lidarSensorResult.error().message);
    return 1;
  }
  auto lidarSensor = std::move(*lidarSensorResult);

  auto odometrySensorResult = ad::scenario::createOdometrySensor(scenario);
  if (!odometrySensorResult) {
    fmt::print(stderr, "Odometry sensor create error: {}\n", odometrySensorResult.error().message);
    return 1;
  }
  auto odometrySensor = std::move(*odometrySensorResult);

  auto physicsResult = ad::scenario::createPhysics(scenario);
  if (!physicsResult) {
    fmt::print(stderr, "Physics create error: {}\n", physicsResult.error().message);
    return 1;
  }
  auto physics = std::move(*physicsResult);

  const ad::simulation::CollisionChecker collisionChecker{map, footprint, scenario.collision};

  const auto odometryDeltaT = scenario.runtime.odometryDeltaT;
  const auto lidarDeltaT = scenario.runtime.lidarDeltaT;
  const auto renderDeltaT = scenario.runtime.renderDeltaT;
  const auto kGoalTolerance = scenario.runtime.goalTolerance;
  const auto kMaxSteps = scenario.runtime.maxSteps;
  auto logFile =
      ad::demo::initializeLogFile(odometryDeltaT, lidarDeltaT, renderDeltaT, kGoalTolerance,
                                  kMaxSteps, footprint.vertices, std::span{*pathResult});
  if (!logFile) {
    return 1;
  }

  const auto initialRenderError = ad::demo::renderFrame(viz, preparedMap, std::span{*pathResult},
                                                        std::nullopt, start, footprint);
  if (initialRenderError) {
    fmt::print(stderr, "Render error: {}\n", initialRenderError->message);
    return 1;
  }

  const auto loopOutcome = ad::demo::runSimulationLoop(
      start, goal, footprint, std::span{*pathResult}, map, collisionChecker, odometryDeltaT,
      lidarDeltaT, renderDeltaT, kGoalTolerance, kMaxSteps, viz, preparedMap, controller, localizer,
      lidarSensor, odometrySensor, physics, *logFile);
  const auto reachedGoal = loopOutcome.reachedGoal;
  const auto &failure = loopOutcome.failure;

  if (failure) {
    fmt::print(stderr, "Simulation error: {}\n", failure->message);
  }

  if (!reachedGoal) {
    fmt::print(stderr, "Simulation ended before reaching the goal.\n");
  }

  *logFile << "# result=" << (reachedGoal && !failure ? "success" : "failure") << '\n';
  if (failure) {
    *logFile << "# error=" << failure->message << '\n';
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
