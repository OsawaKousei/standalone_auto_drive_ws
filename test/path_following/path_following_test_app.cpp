#include "features/control/controller_factory.hpp"
#include "features/planning/planner_factory.hpp"
#include "features/simulation/collision_checker.hpp"
#include "features/simulation/simulation_factory.hpp"
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
#include "shared/text_config.hpp"
#include "shared/types.hpp"

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

namespace ad::path_following_test {

constexpr auto kLogPrecision = 8;
constexpr auto kAnglePeriod = 2.0 * std::numbers::pi;
constexpr auto kRenderScheduleEpsilon = 1.0e-12;

struct AlgorithmSpec {
  std::string algorithm;
  std::string configPath;
};

struct RuntimeConfig {
  double stepSeconds;
  double renderDeltaT;
  int maxSteps;
  double goalTolerance;
};

struct TestScenarioConfig {
  std::string name;
  std::string baseDir;
  std::string mapYamlPath;
  types::Footprint footprint;
  types::Pose start;
  types::Pose goal;
  RuntimeConfig runtime;
  simulation::CollisionCheckConfig collision;
  AlgorithmSpec planning;
  AlgorithmSpec control;
  AlgorithmSpec odometrySensor;
  AlgorithmSpec physics;
};

struct ProgramOptions {
  std::string scenarioPath;
  bool render;
};

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

[[nodiscard]] auto requiredRaw(const config::TextConfig &cfg, std::string_view section,
                               std::string_view key) -> Result<std::string_view> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Required config key is missing: " +
                                                std::string{section} + "." + std::string{key}});
  }
  return *raw;
}

[[nodiscard]] auto requiredString(const config::TextConfig &cfg, std::string_view section,
                                  std::string_view key) -> Result<std::string> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return config::parseQuotedString(*raw);
}

[[nodiscard]] auto requiredDouble(const config::TextConfig &cfg, std::string_view section,
                                  std::string_view key) -> Result<double> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return config::parseDoubleValue(*raw);
}

[[nodiscard]] auto requiredInt(const config::TextConfig &cfg, std::string_view section,
                               std::string_view key) -> Result<int> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return config::parseIntValue(*raw);
}

[[nodiscard]] auto parsePose(const config::TextConfig &cfg, std::string_view section)
    -> Result<types::Pose> {
  const auto xValue = requiredDouble(cfg, section, "x");
  if (!xValue) {
    return tl::make_unexpected(xValue.error());
  }

  const auto yValue = requiredDouble(cfg, section, "y");
  if (!yValue) {
    return tl::make_unexpected(yValue.error());
  }

  const auto thetaValue = requiredDouble(cfg, section, "theta");
  if (!thetaValue) {
    return tl::make_unexpected(thetaValue.error());
  }

  return types::Pose{.x = *xValue, .y = *yValue, .theta = *thetaValue};
}

[[nodiscard]] auto parseFootprintVertices(const config::TextConfig &cfg)
    -> Result<types::Footprint> {
  const auto raw = requiredRaw(cfg, "robot.footprint", "vertices");
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }

  const auto values = config::parseArrayFlat(*raw);
  if (!values) {
    return tl::make_unexpected(values.error());
  }

  if (values->size() < 6U || values->size() % 2U != 0U) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "robot.footprint.vertices must contain N x 2 numeric values."});
  }

  auto vertices = std::vector<types::Point>{};
  vertices.reserve(values->size() / 2U);
  for (std::size_t index = 0; index < values->size(); index += 2U) {
    vertices.push_back(types::Point{.x = (*values)[index], .y = (*values)[index + 1U]});
  }

  return types::Footprint{std::move(vertices)};
}

[[nodiscard]] auto parseAlgorithmSpec(const config::TextConfig &cfg, std::string_view section)
    -> Result<AlgorithmSpec> {
  const auto algorithm = requiredString(cfg, section, "algorithm");
  if (!algorithm) {
    return tl::make_unexpected(algorithm.error());
  }

  const auto configPath = requiredString(cfg, section, "config_path");
  if (!configPath) {
    return tl::make_unexpected(configPath.error());
  }

  return AlgorithmSpec{.algorithm = *algorithm, .configPath = *configPath};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] auto resolvePath(std::string_view baseDir, std::string_view path) -> std::string {
  const auto candidate = std::filesystem::path{std::string{path}};
  if (candidate.is_absolute()) {
    return candidate.lexically_normal().string();
  }
  const auto resolved = std::filesystem::path{std::string{baseDir}} / candidate;
  return resolved.lexically_normal().string();
}

[[nodiscard]] auto loadScenario(std::string_view scenarioPath) -> Result<TestScenarioConfig> {
  const auto scenarioFsPath = std::filesystem::path{std::string{scenarioPath}};
  const auto baseDir = scenarioFsPath.parent_path().empty() ? std::filesystem::path{"."}
                                                            : scenarioFsPath.parent_path();
  const auto baseDirNormalized = baseDir.lexically_normal().string();

  const auto cfg = config::loadTextConfig(scenarioFsPath.lexically_normal().string());
  if (!cfg) {
    return tl::make_unexpected(cfg.error());
  }

  const auto name = requiredString(*cfg, "scenario", "name");
  if (!name) {
    return tl::make_unexpected(name.error());
  }

  const auto mapYamlPath = requiredString(*cfg, "map", "yaml_path");
  if (!mapYamlPath) {
    return tl::make_unexpected(mapYamlPath.error());
  }

  const auto footprint = parseFootprintVertices(*cfg);
  if (!footprint) {
    return tl::make_unexpected(footprint.error());
  }

  const auto start = parsePose(*cfg, "robot.start");
  if (!start) {
    return tl::make_unexpected(start.error());
  }

  const auto goal = parsePose(*cfg, "robot.goal");
  if (!goal) {
    return tl::make_unexpected(goal.error());
  }

  const auto stepSeconds = requiredDouble(*cfg, "simulation.runtime", "step_seconds");
  if (!stepSeconds) {
    return tl::make_unexpected(stepSeconds.error());
  }
  const auto renderDeltaT = requiredDouble(*cfg, "simulation.runtime", "render_delta_t");
  if (!renderDeltaT) {
    return tl::make_unexpected(renderDeltaT.error());
  }
  const auto maxSteps = requiredInt(*cfg, "simulation.runtime", "max_steps");
  if (!maxSteps) {
    return tl::make_unexpected(maxSteps.error());
  }
  const auto goalTolerance = requiredDouble(*cfg, "simulation.runtime", "goal_tolerance");
  if (!goalTolerance) {
    return tl::make_unexpected(goalTolerance.error());
  }

  if (*stepSeconds <= 0.0 || *renderDeltaT <= 0.0 || *maxSteps <= 0 || *goalTolerance <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "simulation.runtime values must be positive for step_seconds, "
                         "render_delta_t, max_steps, and goal_tolerance."});
  }

  const auto maxTranslationStep =
      requiredDouble(*cfg, "simulation.collision", "max_translation_step");
  if (!maxTranslationStep) {
    return tl::make_unexpected(maxTranslationStep.error());
  }
  const auto maxRotationStep = requiredDouble(*cfg, "simulation.collision", "max_rotation_step");
  if (!maxRotationStep) {
    return tl::make_unexpected(maxRotationStep.error());
  }

  const auto planning = parseAlgorithmSpec(*cfg, "planning");
  if (!planning) {
    return tl::make_unexpected(planning.error());
  }

  const auto control = parseAlgorithmSpec(*cfg, "control");
  if (!control) {
    return tl::make_unexpected(control.error());
  }

  const auto odometrySensor = parseAlgorithmSpec(*cfg, "odometry_sensor");
  if (!odometrySensor) {
    return tl::make_unexpected(odometrySensor.error());
  }

  const auto physics = parseAlgorithmSpec(*cfg, "physics");
  if (!physics) {
    return tl::make_unexpected(physics.error());
  }

  return TestScenarioConfig{
      .name = *name,
      .baseDir = baseDirNormalized,
      .mapYamlPath = *mapYamlPath,
      .footprint = *footprint,
      .start = *start,
      .goal = *goal,
      .runtime = RuntimeConfig{.stepSeconds = *stepSeconds,
                               .renderDeltaT = *renderDeltaT,
                               .maxSteps = *maxSteps,
                               .goalTolerance = *goalTolerance},
      .collision = simulation::CollisionCheckConfig{.maxTranslationStep = *maxTranslationStep,
                                                    .maxRotationStep = *maxRotationStep},
      .planning = *planning,
      .control = *control,
      .odometrySensor = *odometrySensor,
      .physics = *physics};
}

[[nodiscard]] auto loadAlgorithmConfig(std::string_view baseDir, std::string_view configPath)
    -> Result<config::TextConfig> {
  return config::loadTextConfig(resolvePath(baseDir, configPath));
}

[[nodiscard]] auto parseProgramOptions(std::span<char *> arguments) -> Result<ProgramOptions> {
  auto scenarioPath = std::string{"test/path_following/configs/path_following.toml"};
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
      return tl::make_unexpected(Error{
          .code = ErrorCode::InvalidInput,
          .message = "Usage: path_following_test_app [scenario.toml] [--render|--no-render]"});
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
  std::filesystem::create_directories("test/path_following/logs", fsError);
  if (fsError) {
    fmt::print(stderr, "Log directory error: {}\n", fsError.message());
    return std::nullopt;
  }

  auto logFile = std::ofstream{"test/path_following/logs/path_following_test.log"};
  if (!logFile.is_open()) {
    fmt::print(stderr, "Log file error: failed to open log file.\n");
    return std::nullopt;
  }

  logFile << std::fixed << std::setprecision(kLogPrecision);
  logFile << "# path_following_test log\n";
  logFile << "# scenario_config=" << scenarioPath << "\n";
  logFile << "# path_point_count=" << path.size() << "\n";
  logFile << "# columns: "
             "step,time,dist_goal,cross_track,nearest_idx,true_x,true_y,true_theta,"
             "odom_x,odom_y,odom_theta,track_err,track_heading_err,cmd_v,cmd_vy,cmd_w,"
             "odom_df,odom_dl,odom_dtheta\n";
  logFile << "step,time,dist_goal,cross_track,nearest_idx,true_x,true_y,true_theta,"
             "odom_x,odom_y,odom_theta,track_err,track_heading_err,cmd_v,cmd_vy,cmd_w,"
             "odom_df,odom_dl,odom_dtheta\n";
  return logFile;
}

} // namespace ad::path_following_test

int main(int argc, char **argv) {
  const auto arguments = std::span<char *>{argv, static_cast<std::size_t>(argc)};
  const auto optionsResult = ad::path_following_test::parseProgramOptions(arguments);
  if (!optionsResult) {
    fmt::print(stderr, "Argument error: {}\n", optionsResult.error().message);
    return 1;
  }
  const auto &options = *optionsResult;
  const auto &scenarioPath = options.scenarioPath;

  const auto scenarioResult = ad::path_following_test::loadScenario(scenarioPath);
  if (!scenarioResult) {
    fmt::print(stderr, "Scenario load error: {}\n", scenarioResult.error().message);
    return 1;
  }
  const auto &scenario = *scenarioResult;

  const auto mapPath = ad::path_following_test::resolvePath(scenario.baseDir, scenario.mapYamlPath);
  const auto mapResult = ad::loadMapFromYaml(mapPath);
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }
  const auto &map = *mapResult;

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

  const auto planningConfig =
      ad::path_following_test::loadAlgorithmConfig(scenario.baseDir, scenario.planning.configPath);
  if (!planningConfig) {
    fmt::print(stderr, "Planning config load error: {}\n", planningConfig.error().message);
    return 1;
  }

  const auto controllerConfig =
      ad::path_following_test::loadAlgorithmConfig(scenario.baseDir, scenario.control.configPath);
  if (!controllerConfig) {
    fmt::print(stderr, "Controller config load error: {}\n", controllerConfig.error().message);
    return 1;
  }

  const auto odometryConfig = ad::path_following_test::loadAlgorithmConfig(
      scenario.baseDir, scenario.odometrySensor.configPath);
  if (!odometryConfig) {
    fmt::print(stderr, "Odometry config load error: {}\n", odometryConfig.error().message);
    return 1;
  }

  const auto physicsConfig =
      ad::path_following_test::loadAlgorithmConfig(scenario.baseDir, scenario.physics.configPath);
  if (!physicsConfig) {
    fmt::print(stderr, "Physics config load error: {}\n", physicsConfig.error().message);
    return 1;
  }

  auto plannerResult =
      ad::planning::createPlannerFromConfig(scenario.planning.algorithm, map, scenario.footprint,
                                            std::optional<ad::config::TextConfig>{*planningConfig});
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

  auto controllerResult = ad::control::createControllerFromConfig(
      scenario.control.algorithm, std::optional<ad::config::TextConfig>{*controllerConfig});
  if (!controllerResult) {
    fmt::print(stderr, "Controller create error: {}\n", controllerResult.error().message);
    return 1;
  }
  auto controller = std::move(*controllerResult);

  auto odometrySensorResult = ad::simulation::createOdometrySensorFromConfig(
      scenario.odometrySensor.algorithm, std::optional<ad::config::TextConfig>{*odometryConfig});
  if (!odometrySensorResult) {
    fmt::print(stderr, "Odometry sensor create error: {}\n", odometrySensorResult.error().message);
    return 1;
  }
  auto odometrySensor = std::move(*odometrySensorResult);

  auto physicsResult = ad::simulation::createPhysicsFromConfig(
      scenario.physics.algorithm, std::optional<ad::config::TextConfig>{*physicsConfig});
  if (!physicsResult) {
    fmt::print(stderr, "Physics create error: {}\n", physicsResult.error().message);
    return 1;
  }
  auto physics = std::move(*physicsResult);

  const ad::simulation::CollisionChecker collisionChecker{map, scenario.footprint,
                                                          scenario.collision};

  auto logFile = ad::path_following_test::initializeLogFile(std::span{*pathResult}, scenarioPath);
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
        ad::path_following_test::renderFrame(*visualizer, *preparedMap, std::span{*pathResult},
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
    odometryPose.emplace(ad::path_following_test::integrateOdometry(*odometryPose, *odometryDelta));

    const auto distGoal = ad::path_following_test::distanceToGoal(trueState->pose, scenario.goal);
    const auto crossTrack =
        ad::path_following_test::minDistanceToPath(trueState->pose, std::span{*pathResult});
    const auto nearestIndex =
        ad::path_following_test::nearestPathIndex(*odometryPose, std::span{*pathResult});
    const auto trackingError =
        std::hypot(odometryPose->x - trueState->pose.x, odometryPose->y - trueState->pose.y);
    const auto trackingHeadingError = std::abs(
        ad::path_following_test::normalizeAngle(odometryPose->theta - trueState->pose.theta));

    *logFile << step << ',' << (scenario.runtime.stepSeconds * static_cast<double>(step + 1)) << ','
             << distGoal << ',' << crossTrack << ',' << nearestIndex << ',' << trueState->pose.x
             << ',' << trueState->pose.y << ',' << trueState->pose.theta << ',' << odometryPose->x
             << ',' << odometryPose->y << ',' << odometryPose->theta << ',' << trackingError << ','
             << trackingHeadingError << ',' << commandResult->v << ',' << commandResult->vy << ','
             << commandResult->w << ',' << odometryDelta->deltaForward << ','
             << odometryDelta->deltaLateral << ',' << odometryDelta->deltaTheta << '\n';

    if (options.render) {
      renderElapsed += scenario.runtime.stepSeconds;
      if (renderElapsed + ad::path_following_test::kRenderScheduleEpsilon >=
          scenario.runtime.renderDeltaT) {
        const auto renderError = ad::path_following_test::renderFrame(
            *visualizer, *preparedMap, std::span{*pathResult}, trueState->pose, scenario.goal,
            scenario.footprint);
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
    fmt::print(stderr, "Path following error: {}\n", failure->message);
  }

  if (!reachedGoal) {
    fmt::print(stderr, "Path following test ended before reaching the goal.\n");
    return 1;
  }

  fmt::print("Path following test reached goal. Log: "
             "test/path_following/logs/path_following_test.log\n");
  return 0;
}
