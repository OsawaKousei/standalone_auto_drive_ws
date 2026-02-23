#include "features/simulation/collision_checker/collision_checker.hpp"
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

#include <array>
#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ad::control_test {

constexpr auto kLogPrecision = 8;
constexpr auto kAnglePeriod = 2.0 * std::numbers::pi;
constexpr auto kRenderScheduleEpsilon = 1.0e-12;

struct ProgramOptions {
  std::string scenarioPath;
  bool render;
};

auto printControlParameters(const scenario::ScenarioConfig &scenario, const types::MapData &map)
    -> void {
  fmt::print("=== Control Test Parameters ===\n");
  fmt::print("scenario.name={}\n", scenario.name);
  fmt::print("map.yaml_path={}\n", scenario.mapYamlPath);
  fmt::print("map.size=({}, {})\n", map.width, map.height);
  fmt::print("map.resolution={}\n", map.resolution);

  fmt::print("robot.start=(x={}, y={}, theta={})\n", scenario.start.x, scenario.start.y,
             scenario.start.theta);
  fmt::print("robot.goal=(x={}, y={}, theta={})\n", scenario.goal.x, scenario.goal.y,
             scenario.goal.theta);
  fmt::print("robot.footprint.vertex_count={}\n", scenario.footprint.vertices.size());

  fmt::print("simulation.runtime.step_seconds={}\n", scenario.runtime.stepSeconds);
  fmt::print("simulation.runtime.render_delta_t={}\n", scenario.runtime.renderDeltaT);
  fmt::print("simulation.runtime.max_steps={}\n", scenario.runtime.maxSteps);
  fmt::print("simulation.runtime.goal_tolerance={}\n", scenario.runtime.goalTolerance);

  fmt::print("simulation.collision.max_translation_step={}\n",
             scenario.collision.maxTranslationStep);
  fmt::print("simulation.collision.max_rotation_step={}\n", scenario.collision.maxRotationStep);

  const auto printAlgorithmSpec = [](std::string_view section,
                                     const scenario::AlgorithmSpec &spec) -> void {
    fmt::print("{}.algorithm={}\n", section, spec.algorithm);
    if (spec.configPath.has_value()) {
      fmt::print("{}.config_path={}\n", section, *spec.configPath);
    } else {
      fmt::print("{}.config_path=<default>\n", section);
    }
  };

  printAlgorithmSpec("planning", scenario.planning);
  printAlgorithmSpec("control", scenario.control);
  printAlgorithmSpec("odometry_sensor", scenario.odometrySensor);
  printAlgorithmSpec("physics", scenario.physics);

  const auto printConfigValues = [](std::string_view section,
                                    const std::optional<config::TextConfig> &doc,
                                    std::span<const std::string_view> keys) -> void {
    if (!doc.has_value()) {
      fmt::print("{}.config=<none>\n", section);
      return;
    }

    for (const auto key : keys) {
      const auto value = doc->findRaw("", key);
      if (!value) {
        continue;
      }
      fmt::print("{}.{}={}\n", section, key, *value);
    }
  };

  constexpr auto kPlanningKeys = std::array<std::string_view, 9>{
      "collision_checker", "resolution",       "allow_diagonal", "weight",        "cost_straight",
      "cost_diagonal",     "inflation_radius", "goal_tolerance", "max_iterations"};
  constexpr auto kControlKeys =
      std::array<std::string_view, 9>{"lookahead_distance", "desired_linear_velocity",
                                      "max_linear_speed",   "max_angular_speed",
                                      "position_kp",        "position_ki",
                                      "position_kd",        "heading_kp",
                                      "heading_kd"};
  constexpr auto kOdometryKeys = std::array<std::string_view, 3>{
      "forward_noise_stddev", "lateral_noise_stddev", "theta_noise_stddev"};
  constexpr auto kPhysicsKeys = std::array<std::string_view, 6>{
      "max_linear_speed",         "max_angular_speed", "max_linear_acceleration",
      "max_angular_acceleration", "tau_linear",        "tau_angular"};

  printConfigValues("planning", scenario.algorithmConfigDocs.planning, kPlanningKeys);
  printConfigValues("control", scenario.algorithmConfigDocs.control, kControlKeys);
  printConfigValues("odometry_sensor", scenario.algorithmConfigDocs.odometrySensor, kOdometryKeys);
  printConfigValues("physics", scenario.algorithmConfigDocs.physics, kPhysicsKeys);
  fmt::print("===============================\n");
}

[[nodiscard]] auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, kAnglePeriod);
  if (angle < 0.0) {
    angle += kAnglePeriod;
  }
  return angle - std::numbers::pi;
}

[[nodiscard]] auto distanceToGoal(const types::Pose &pose, const types::Pose &goal) -> double {
  return std::hypot(goal.x - pose.x, goal.y - pose.y);
}

[[nodiscard]] auto serializePoints(std::span<const types::Point> points) -> std::string {
  auto output = std::string{};
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (index != 0U) {
      output.push_back(';');
    }
    output += fmt::format("{:.8f}:{:.8f}", points[index].x, points[index].y);
  }
  return output;
}

[[nodiscard]] auto nearestPathIndex(const types::Pose &pose, std::span<const types::Point> path)
    -> std::size_t {
  auto nearestIndex = std::size_t{0};
  auto bestDistance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const auto dx = path[index].x - pose.x;
    const auto dy = path[index].y - pose.y;
    const auto distance = std::hypot(dx, dy);
    if (distance < bestDistance) {
      bestDistance = distance;
      nearestIndex = index;
    }
  }
  return nearestIndex;
}

[[nodiscard]] auto minDistanceToPath(const types::Pose &pose, std::span<const types::Point> path)
    -> double {
  if (path.empty()) {
    return 0.0;
  }
  const auto index = nearestPathIndex(pose, path);
  return std::hypot(path[index].x - pose.x, path[index].y - pose.y);
}

[[nodiscard]] auto integrateOdometry(const types::Pose &currentPose,
                                     const types::OdometryDelta &delta) -> types::Pose {
  const auto cosTheta = std::cos(currentPose.theta);
  const auto sinTheta = std::sin(currentPose.theta);
  const auto deltaX = (cosTheta * delta.deltaForward) - (sinTheta * delta.deltaLateral);
  const auto deltaY = (sinTheta * delta.deltaForward) + (cosTheta * delta.deltaLateral);

  return types::Pose{.x = currentPose.x + deltaX,
                     .y = currentPose.y + deltaY,
                     .theta = normalizeAngle(currentPose.theta + delta.deltaTheta)};
}

[[nodiscard]] auto parseProgramOptions(std::span<char *> arguments) -> Result<ProgramOptions> {
  auto scenarioPath = std::string{"test/control/configs/scenario.toml"};
  auto render = false;

  for (std::size_t index = 1; index < arguments.size(); ++index) {
    const auto argument = std::string_view{arguments[index]};
    if (argument == "--render") {
      render = true;
      continue;
    }
    if (argument == "--no-render") {
      render = false;
      continue;
    }
    if (argument == "-h" || argument == "--help") {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput,
                .message = "Usage: control_test_app [scenario.toml] [--render|--no-render]"});
    }
    if (!argument.empty() && argument.front() == '-') {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Unknown option: " + std::string{argument}});
    }
    scenarioPath = std::string{argument};
  }

  return ProgramOptions{.scenarioPath = scenarioPath, .render = render};
}

[[nodiscard]] auto renderFrame(const ad::visualization::Visualizer &viz,
                               const ad::visualization::Visualizer::PreparedMap &preparedMap,
                               std::span<const types::Point> path, const types::Pose &robotPose,
                               const types::Pose &goal, const types::Footprint &footprint)
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

  const auto goalStatus =
      viz.renderMarker(types::Point{.x = goal.x, .y = goal.y}, mapGeometry, 18.0, "red");
  if (!goalStatus) {
    return goalStatus.error();
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

[[nodiscard]] auto initializeLogFile(std::span<const types::Point> path,
                                     std::string_view scenarioPath)
    -> std::optional<std::ofstream> {
  std::error_code fsError;
  std::filesystem::create_directories("test/control/logs", fsError);
  if (fsError) {
    fmt::print(stderr, "Log directory error: {}\n", fsError.message());
    return std::nullopt;
  }

  auto logFile = std::ofstream{"test/control/logs/control_test.log"};
  if (!logFile.is_open()) {
    fmt::print(stderr, "Log file error: failed to open log file.\n");
    return std::nullopt;
  }

  logFile << std::fixed << std::setprecision(kLogPrecision);
  logFile << "# control_test log\n";
  logFile << "# scenario_config=" << scenarioPath << "\n";
  logFile << "# path_point_count=" << path.size() << "\n";
  logFile << "# path=" << serializePoints(path) << "\n";
  logFile << "# columns: "
             "step,time,dist_goal,cross_track,nearest_idx,true_x,true_y,true_theta,"
             "odom_x,odom_y,odom_theta,track_err,track_heading_err,cmd_v,cmd_vy,cmd_w,"
             "odom_df,odom_dl,odom_dtheta\n";
  logFile << "step,time,dist_goal,cross_track,nearest_idx,true_x,true_y,true_theta,"
             "odom_x,odom_y,odom_theta,track_err,track_heading_err,cmd_v,cmd_vy,cmd_w,"
             "odom_df,odom_dl,odom_dtheta\n";
  return logFile;
}

} // namespace ad::control_test

int main(int argc, char **argv) {
  const auto arguments = std::span<char *>{argv, static_cast<std::size_t>(argc)};
  const auto optionsResult = ad::control_test::parseProgramOptions(arguments);
  if (!optionsResult) {
    fmt::print(stderr, "Argument error: {}\n", optionsResult.error().message);
    return 1;
  }
  const auto &options = *optionsResult;
  const auto &scenarioPath = options.scenarioPath;

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
  const auto &map = *mapResult;

  ad::control_test::printControlParameters(scenario, map);

  auto visualizer = std::optional<ad::visualization::Visualizer>{};
  auto preparedMap = std::optional<ad::visualization::Visualizer::PreparedMap>{};
  if (options.render) {
    const auto preparedMapResult = ad::visualization::Visualizer::prepareMap(map);
    if (!preparedMapResult) {
      fmt::print(stderr, "Render error: {}\n", preparedMapResult.error().message);
      return 1;
    }
    visualizer.emplace();
    preparedMap.emplace(*preparedMapResult);
  }

  auto plannerResult = ad::scenario::createPlanner(scenario, map, scenario.footprint);
  if (!plannerResult) {
    fmt::print(stderr, "Planner create error: {}\n", plannerResult.error().message);
    return 1;
  }
  auto planner = std::move(*plannerResult);

  const auto pathResult = planner->plan(map, scenario.start, scenario.goal, scenario.footprint);
  if (!pathResult) {
    fmt::print(stderr, "Planning error: {}\n", pathResult.error().message);
    return 1;
  }

  auto controllerResult = ad::scenario::createController(scenario);
  if (!controllerResult) {
    fmt::print(stderr, "Controller create error: {}\n", controllerResult.error().message);
    return 1;
  }
  auto controller = std::move(*controllerResult);

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

  const ad::simulation::CollisionChecker collisionChecker{map, scenario.footprint,
                                                          scenario.collision};

  auto logFile = ad::control_test::initializeLogFile(std::span{*pathResult}, scenarioPath);
  if (!logFile) {
    return 1;
  }

  auto trueState = std::optional<ad::simulation::MotionState>{ad::simulation::MotionState{
      .pose = scenario.start, .twist = ad::types::Twist{.v = 0.0, .vy = 0.0, .w = 0.0}}};
  auto odometryPose = std::optional<ad::types::Pose>{scenario.start};
  auto reachedGoal = false;
  std::optional<ad::Error> failure;
  auto renderElapsed = 0.0;

  if (options.render) {
    const auto renderError =
        ad::control_test::renderFrame(*visualizer, *preparedMap, std::span{*pathResult},
                                      trueState->pose, scenario.goal, scenario.footprint);
    if (renderError) {
      fmt::print(stderr, "Render error: {}\n", renderError->message);
      return 1;
    }
  }

  for (int step = 0; step < scenario.runtime.maxSteps; ++step) {
    const auto commandResult = controller->computeCommand(
        ad::control::ControlInput{.path = std::span{*pathResult},
                                  .currentPose = *odometryPose,
                                  .deltaSeconds = scenario.runtime.stepSeconds});
    if (!commandResult) {
      failure.emplace(commandResult.error());
      break;
    }

    const auto nextState =
        physics->propagate(*trueState, *commandResult, scenario.runtime.stepSeconds);
    if (!nextState) {
      failure.emplace(nextState.error());
      break;
    }

    const auto collisionStatus = collisionChecker.checkTrajectory(trueState->pose, nextState->pose);
    if (!collisionStatus) {
      failure.emplace(collisionStatus.error());
      break;
    }
    if (!*collisionStatus) {
      failure.emplace(ad::Error{.code = ad::ErrorCode::InvalidInput,
                                .message = "Collision detected in trajectory propagation."});
      break;
    }

    const auto odometryDelta = odometrySensor->measure(trueState->pose, nextState->pose);
    if (!odometryDelta) {
      failure.emplace(odometryDelta.error());
      break;
    }

    trueState.emplace(
        ad::simulation::MotionState{.pose = nextState->pose, .twist = nextState->twist});
    odometryPose.emplace(ad::control_test::integrateOdometry(*odometryPose, *odometryDelta));

    const auto distGoal = ad::control_test::distanceToGoal(trueState->pose, scenario.goal);
    const auto crossTrack =
        ad::control_test::minDistanceToPath(trueState->pose, std::span{*pathResult});
    const auto nearestIndex =
        ad::control_test::nearestPathIndex(*odometryPose, std::span{*pathResult});
    const auto trackingError =
        std::hypot(odometryPose->x - trueState->pose.x, odometryPose->y - trueState->pose.y);
    const auto trackingHeadingError =
        std::abs(ad::control_test::normalizeAngle(odometryPose->theta - trueState->pose.theta));

    *logFile << step << ',' << (scenario.runtime.stepSeconds * static_cast<double>(step + 1)) << ','
             << distGoal << ',' << crossTrack << ',' << nearestIndex << ',' << trueState->pose.x
             << ',' << trueState->pose.y << ',' << trueState->pose.theta << ',' << odometryPose->x
             << ',' << odometryPose->y << ',' << odometryPose->theta << ',' << trackingError << ','
             << trackingHeadingError << ',' << commandResult->v << ',' << commandResult->vy << ','
             << commandResult->w << ',' << odometryDelta->deltaForward << ','
             << odometryDelta->deltaLateral << ',' << odometryDelta->deltaTheta << '\n';

    if (options.render) {
      renderElapsed += scenario.runtime.stepSeconds;
      if (renderElapsed + ad::control_test::kRenderScheduleEpsilon >=
          scenario.runtime.renderDeltaT) {
        const auto renderError =
            ad::control_test::renderFrame(*visualizer, *preparedMap, std::span{*pathResult},
                                          trueState->pose, scenario.goal, scenario.footprint);
        if (renderError) {
          failure.emplace(*renderError);
          break;
        }
        renderElapsed = std::fmod(renderElapsed, scenario.runtime.renderDeltaT);
      }
    }

    if (distGoal <= scenario.runtime.goalTolerance) {
      reachedGoal = true;
      break;
    }
  }

  *logFile << "# result=" << (reachedGoal && !failure ? "success" : "failure") << '\n';
  if (failure) {
    *logFile << "# error=" << failure->message << '\n';
    fmt::print(stderr, "Control test error: {}\n", failure->message);
  }

  if (!reachedGoal) {
    fmt::print(stderr, "Control test ended before reaching the goal.\n");
    return 1;
  }

  fmt::print("Control test reached goal. Log: test/control/logs/control_test.log\n");
  return 0;
}
