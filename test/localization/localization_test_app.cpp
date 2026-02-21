#include "features/control/controller_factory.hpp"
#include "features/localization/localizer_factory.hpp"
#include "features/planning/planner_factory.hpp"
#include "features/simulation/collision_checker/collision_checker.hpp"
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

namespace ad::localization_test {

constexpr auto kLogPrecision = 8;
constexpr auto kAnglePeriod = 2.0 * std::numbers::pi;
constexpr auto kScheduleEpsilon = 1.0e-12;

struct AlgorithmSpec {
  std::string algorithm;
  std::string configPath;
};

struct RuntimeConfig {
  double stepSeconds;
  double lidarDeltaT;
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
  AlgorithmSpec localization;
  AlgorithmSpec planning;
  AlgorithmSpec control;
  AlgorithmSpec lidarSensor;
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

[[nodiscard]] auto serializeValues(std::span<const double> values) -> std::string {
  auto output = std::string{};
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output.push_back(';');
    }
    output += fmt::format("{:.8f}", values[index]);
  }
  return output;
}

[[nodiscard]] auto scanToRobotPoints(const types::LidarScan &scan) -> std::vector<types::Point> {
  auto points = std::vector<types::Point>{};
  points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    const auto range = scan.ranges[index];
    points.push_back(types::Point{.x = range * std::cos(angle), .y = range * std::sin(angle)});
  }
  return points;
}

[[nodiscard]] auto scanToWorldPoints(const types::Pose &pose, const types::LidarScan &scan)
    -> std::vector<types::Point> {
  auto points = std::vector<types::Point>{};
  points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto angle =
        pose.theta + scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    const auto range = scan.ranges[index];
    points.push_back(types::Point{.x = pose.x + (range * std::cos(angle)),
                                  .y = pose.y + (range * std::sin(angle))});
  }
  return points;
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
  const auto lidarDeltaT = requiredDouble(*cfg, "simulation.runtime", "lidar_delta_t");
  if (!lidarDeltaT) {
    return tl::make_unexpected(lidarDeltaT.error());
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

  if (*stepSeconds <= 0.0 || *lidarDeltaT <= 0.0 || *renderDeltaT <= 0.0 ||
      *lidarDeltaT < *stepSeconds || *maxSteps <= 0 || *goalTolerance <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "simulation.runtime values are invalid for step_seconds, lidar_delta_t, "
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

  const auto localization = parseAlgorithmSpec(*cfg, "localization");
  if (!localization) {
    return tl::make_unexpected(localization.error());
  }

  const auto planning = parseAlgorithmSpec(*cfg, "planning");
  if (!planning) {
    return tl::make_unexpected(planning.error());
  }

  const auto control = parseAlgorithmSpec(*cfg, "control");
  if (!control) {
    return tl::make_unexpected(control.error());
  }

  const auto lidarSensor = parseAlgorithmSpec(*cfg, "lidar_sensor");
  if (!lidarSensor) {
    return tl::make_unexpected(lidarSensor.error());
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
                               .lidarDeltaT = *lidarDeltaT,
                               .renderDeltaT = *renderDeltaT,
                               .maxSteps = *maxSteps,
                               .goalTolerance = *goalTolerance},
      .collision = simulation::CollisionCheckConfig{.maxTranslationStep = *maxTranslationStep,
                                                    .maxRotationStep = *maxRotationStep},
      .localization = *localization,
      .planning = *planning,
      .control = *control,
      .lidarSensor = *lidarSensor,
      .odometrySensor = *odometrySensor,
      .physics = *physics};
}

[[nodiscard]] auto loadAlgorithmConfig(std::string_view baseDir, std::string_view configPath)
    -> Result<config::TextConfig> {
  return config::loadTextConfig(resolvePath(baseDir, configPath));
}

[[nodiscard]] auto parseProgramOptions(std::span<char *> arguments) -> Result<ProgramOptions> {
  auto scenarioPath = std::string{"test/localization/configs/localization.toml"};
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
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Usage: localization_test_app [scenario.toml] "
                                                  "[--render|--no-render]"});
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
                               const types::Footprint &footprint,
                               const std::optional<std::vector<types::Point>> &scanWorldPoints)
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

[[nodiscard]] auto initializeLogFile(std::span<const types::Point> path,
                                     std::string_view scenarioPath)
    -> std::optional<std::ofstream> {
  std::error_code fsError;
  std::filesystem::create_directories("test/localization/logs", fsError);
  if (fsError) {
    fmt::print(stderr, "Log directory error: {}\n", fsError.message());
    return std::nullopt;
  }

  auto logFile = std::ofstream{"test/localization/logs/localization_test.log"};
  if (!logFile.is_open()) {
    fmt::print(stderr, "Log file error: failed to open localization test log file.\n");
    return std::nullopt;
  }

  logFile << std::fixed << std::setprecision(kLogPrecision);
  logFile << "# localization_test log\n";
  logFile << "# scenario_config=" << scenarioPath << "\n";
  logFile << "# path_point_count=" << path.size() << "\n";
  logFile << "# path=" << serializePoints(path) << "\n";
  logFile << "# columns: "
             "step,time,dist_goal,true_x,true_y,true_theta,est_x,est_y,est_theta,pos_err,"
             "heading_err,pre_update_pos_err,post_update_pos_err,delta_pos_err,"
             "pre_update_heading_err,post_update_heading_err,delta_heading_err,score,cov_xx,"
             "cov_yy,cov_tt,cmd_v,cmd_vy,cmd_w,odom_df,odom_dl,odom_dtheta,lidar_updated,"
             "scan_count,scan_min_angle,scan_angle_inc,scan_max_range,scan_ranges,"
             "scan_points_robot\n";
  logFile << "step,time,dist_goal,true_x,true_y,true_theta,est_x,est_y,est_theta,pos_err,"
             "heading_err,pre_update_pos_err,post_update_pos_err,delta_pos_err,"
             "pre_update_heading_err,post_update_heading_err,delta_heading_err,score,cov_xx,"
             "cov_yy,cov_tt,cmd_v,cmd_vy,cmd_w,odom_df,odom_dl,odom_dtheta,lidar_updated,"
             "scan_count,scan_min_angle,scan_angle_inc,scan_max_range,scan_ranges,"
             "scan_points_robot\n";
  return logFile;
}

} // namespace ad::localization_test

int main(int argc, char **argv) {
  const auto arguments = std::span<char *>{argv, static_cast<std::size_t>(argc)};
  const auto optionsResult = ad::localization_test::parseProgramOptions(arguments);
  if (!optionsResult) {
    fmt::print(stderr, "Argument error: {}\n", optionsResult.error().message);
    return 1;
  }

  const auto &scenarioPath = optionsResult->scenarioPath;
  const auto scenarioResult = ad::localization_test::loadScenario(scenarioPath);
  if (!scenarioResult) {
    fmt::print(stderr, "Scenario load error: {}\n", scenarioResult.error().message);
    return 1;
  }
  const auto &scenario = *scenarioResult;

  const auto mapPath = ad::localization_test::resolvePath(scenario.baseDir, scenario.mapYamlPath);
  const auto mapResult = ad::loadMapFromYaml(mapPath);
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }
  const auto &map = *mapResult;

  auto visualizer = std::optional<ad::visualization::Visualizer>{};
  auto preparedMap = std::optional<ad::visualization::Visualizer::PreparedMap>{};
  auto *visualizerPtr = static_cast<ad::visualization::Visualizer *>(nullptr);
  auto *preparedMapPtr = static_cast<ad::visualization::Visualizer::PreparedMap *>(nullptr);
  if (optionsResult->render) {
    const auto preparedMapResult = ad::visualization::Visualizer::prepareMap(map);
    if (!preparedMapResult) {
      fmt::print(stderr, "Render error: {}\n", preparedMapResult.error().message);
      return 1;
    }
    visualizer.emplace();
    preparedMap.emplace(*preparedMapResult);
    visualizerPtr = &visualizer.value();
    preparedMapPtr = &preparedMap.value();
  }

  const auto localizationConfig = ad::localization_test::loadAlgorithmConfig(
      scenario.baseDir, scenario.localization.configPath);
  if (!localizationConfig) {
    fmt::print(stderr, "Localization config load error: {}\n", localizationConfig.error().message);
    return 1;
  }
  const auto planningConfig =
      ad::localization_test::loadAlgorithmConfig(scenario.baseDir, scenario.planning.configPath);
  if (!planningConfig) {
    fmt::print(stderr, "Planning config load error: {}\n", planningConfig.error().message);
    return 1;
  }
  const auto controllerConfig =
      ad::localization_test::loadAlgorithmConfig(scenario.baseDir, scenario.control.configPath);
  if (!controllerConfig) {
    fmt::print(stderr, "Controller config load error: {}\n", controllerConfig.error().message);
    return 1;
  }
  const auto lidarConfig =
      ad::localization_test::loadAlgorithmConfig(scenario.baseDir, scenario.lidarSensor.configPath);
  if (!lidarConfig) {
    fmt::print(stderr, "Lidar config load error: {}\n", lidarConfig.error().message);
    return 1;
  }
  const auto odometryConfig = ad::localization_test::loadAlgorithmConfig(
      scenario.baseDir, scenario.odometrySensor.configPath);
  if (!odometryConfig) {
    fmt::print(stderr, "Odometry config load error: {}\n", odometryConfig.error().message);
    return 1;
  }
  const auto physicsConfig =
      ad::localization_test::loadAlgorithmConfig(scenario.baseDir, scenario.physics.configPath);
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

  const auto pathResult =
      (*plannerResult)->plan(map, scenario.start, scenario.goal, scenario.footprint);
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

  auto localizerResult = ad::localization::createLocalizerFromConfig(
      scenario.localization.algorithm, map,
      std::optional<ad::config::TextConfig>{*localizationConfig});
  if (!localizerResult) {
    fmt::print(stderr, "Localizer create error: {}\n", localizerResult.error().message);
    return 1;
  }

  const auto initialCovariance = ad::localization::parseInitialCovarianceFromConfig(
      std::optional<ad::config::TextConfig>{*localizationConfig});
  if (!initialCovariance) {
    fmt::print(stderr, "Initial covariance error: {}\n", initialCovariance.error().message);
    return 1;
  }

  const auto resetStatus = (*localizerResult)->reset(scenario.start, *initialCovariance);
  if (!resetStatus) {
    fmt::print(stderr, "Localizer reset error: {}\n", resetStatus.error().message);
    return 1;
  }

  auto lidarSensorResult = ad::simulation::createLidarSensorFromConfig(
      scenario.lidarSensor.algorithm, std::optional<ad::config::TextConfig>{*lidarConfig});
  if (!lidarSensorResult) {
    fmt::print(stderr, "Lidar sensor create error: {}\n", lidarSensorResult.error().message);
    return 1;
  }

  auto odometrySensorResult = ad::simulation::createOdometrySensorFromConfig(
      scenario.odometrySensor.algorithm, std::optional<ad::config::TextConfig>{*odometryConfig});
  if (!odometrySensorResult) {
    fmt::print(stderr, "Odometry sensor create error: {}\n", odometrySensorResult.error().message);
    return 1;
  }

  auto physicsResult = ad::simulation::createPhysicsFromConfig(
      scenario.physics.algorithm, std::optional<ad::config::TextConfig>{*physicsConfig});
  if (!physicsResult) {
    fmt::print(stderr, "Physics create error: {}\n", physicsResult.error().message);
    return 1;
  }

  const ad::simulation::CollisionChecker collisionChecker{map, scenario.footprint,
                                                          scenario.collision};

  auto logFile = ad::localization_test::initializeLogFile(std::span{*pathResult}, scenarioPath);
  if (!logFile) {
    return 1;
  }

  auto trueState = std::optional<ad::simulation::MotionState>{ad::simulation::MotionState{
      .pose = scenario.start, .twist = ad::types::Twist{.v = 0.0, .vy = 0.0, .w = 0.0}}};
  auto reachedGoal = false;
  std::optional<ad::Error> failure;
  auto lidarElapsed = 0.0;
  auto renderElapsed = 0.0;
  auto lastScanWorldPoints = std::optional<std::vector<ad::types::Point>>{};

  if (optionsResult->render) {
    const auto renderError =
        ad::localization_test::renderFrame(*visualizerPtr, *preparedMapPtr, std::span{*pathResult},
                                           trueState->pose, scenario.footprint, std::nullopt);
    if (renderError) {
      fmt::print(stderr, "Render error: {}\n", renderError->message);
      return 1;
    }
  }

  for (int step = 0; step < scenario.runtime.maxSteps; ++step) {
    const auto estimateBefore = (*localizerResult)->estimate();
    if (!estimateBefore) {
      failure.emplace(estimateBefore.error());
      break;
    }

    const auto commandResult = (*controllerResult)
                                   ->computeCommand(ad::control::ControlInput{
                                       .path = std::span{*pathResult},
                                       .currentPose = estimateBefore->pose,
                                       .deltaSeconds = scenario.runtime.stepSeconds});
    if (!commandResult) {
      failure.emplace(commandResult.error());
      break;
    }

    const auto nextState =
        (*physicsResult)->propagate(*trueState, *commandResult, scenario.runtime.stepSeconds);
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

    const auto odometryDelta = (*odometrySensorResult)->measure(trueState->pose, nextState->pose);
    if (!odometryDelta) {
      failure.emplace(odometryDelta.error());
      break;
    }

    const auto predictStatus = (*localizerResult)->predictOdometry(*odometryDelta);
    if (!predictStatus) {
      failure.emplace(predictStatus.error());
      break;
    }

    trueState.emplace(
        ad::simulation::MotionState{.pose = nextState->pose, .twist = nextState->twist});

    const auto estimateBeforeUpdate = (*localizerResult)->estimate();
    if (!estimateBeforeUpdate) {
      failure.emplace(estimateBeforeUpdate.error());
      break;
    }

    const auto prePositionError = std::hypot(estimateBeforeUpdate->pose.x - trueState->pose.x,
                                             estimateBeforeUpdate->pose.y - trueState->pose.y);
    const auto preHeadingError = std::abs(ad::localization_test::normalizeAngle(
        estimateBeforeUpdate->pose.theta - trueState->pose.theta));

    auto lidarUpdated = false;
    auto scanCount = 0;
    auto scanMinAngle = 0.0;
    auto scanAngleIncrement = 0.0;
    auto scanMaxRange = 0.0;
    auto scanRangesSerialized = std::string{};
    auto scanPointsRobotSerialized = std::string{};

    lidarElapsed += scenario.runtime.stepSeconds;
    if (lidarElapsed + ad::localization_test::kScheduleEpsilon >= scenario.runtime.lidarDeltaT) {
      const auto scanResult = (*lidarSensorResult)->simulate(map, trueState->pose);
      if (!scanResult) {
        failure.emplace(scanResult.error());
        break;
      }

      const auto updateStatus = (*localizerResult)->update(*scanResult, map);
      if (!updateStatus) {
        failure.emplace(updateStatus.error());
        break;
      }

      const auto robotScanPoints = ad::localization_test::scanToRobotPoints(*scanResult);
      scanCount = static_cast<int>(scanResult->ranges.size());
      scanMinAngle = scanResult->minAngle;
      scanAngleIncrement = scanResult->angleIncrement;
      scanMaxRange = scanResult->maxRange;
      scanRangesSerialized = ad::localization_test::serializeValues(scanResult->ranges);
      scanPointsRobotSerialized = ad::localization_test::serializePoints(robotScanPoints);
      lastScanWorldPoints.emplace(
          ad::localization_test::scanToWorldPoints(trueState->pose, *scanResult));
      lidarUpdated = true;
      lidarElapsed = std::fmod(lidarElapsed, scenario.runtime.lidarDeltaT);
    }

    if (optionsResult->render) {
      renderElapsed += scenario.runtime.stepSeconds;
      const auto shouldRender =
          lidarUpdated || (renderElapsed + ad::localization_test::kScheduleEpsilon >=
                           scenario.runtime.renderDeltaT);
      if (shouldRender) {
        const auto renderError = ad::localization_test::renderFrame(
            *visualizerPtr, *preparedMapPtr, std::span{*pathResult}, trueState->pose,
            scenario.footprint, lastScanWorldPoints);
        if (renderError) {
          failure.emplace(*renderError);
          break;
        }
        renderElapsed = std::fmod(renderElapsed, scenario.runtime.renderDeltaT);
      }
    }

    const auto estimateAfter = (*localizerResult)->estimate();
    if (!estimateAfter) {
      failure.emplace(estimateAfter.error());
      break;
    }

    const auto postPositionError = std::hypot(estimateAfter->pose.x - trueState->pose.x,
                                              estimateAfter->pose.y - trueState->pose.y);
    const auto postHeadingError = std::abs(
        ad::localization_test::normalizeAngle(estimateAfter->pose.theta - trueState->pose.theta));
    const auto deltaPositionError = lidarUpdated ? (prePositionError - postPositionError)
                                                 : std::numeric_limits<double>::quiet_NaN();
    const auto deltaHeadingError = lidarUpdated ? (preHeadingError - postHeadingError)
                                                : std::numeric_limits<double>::quiet_NaN();

    const auto distanceGoal = ad::localization_test::distanceToGoal(trueState->pose, scenario.goal);
    const auto timeSeconds = scenario.runtime.stepSeconds * static_cast<double>(step + 1);

    *logFile << step << ',' << timeSeconds << ',' << distanceGoal << ',' << trueState->pose.x << ','
             << trueState->pose.y << ',' << trueState->pose.theta << ',' << estimateAfter->pose.x
             << ',' << estimateAfter->pose.y << ',' << estimateAfter->pose.theta << ','
             << postPositionError << ',' << postHeadingError << ',' << prePositionError << ','
             << postPositionError << ',' << deltaPositionError << ',' << preHeadingError << ','
             << postHeadingError << ',' << deltaHeadingError << ',' << estimateAfter->score << ','
             << estimateAfter->covariance(0, 0) << ',' << estimateAfter->covariance(1, 1) << ','
             << estimateAfter->covariance(2, 2) << ',' << commandResult->v << ','
             << commandResult->vy << ',' << commandResult->w << ',' << odometryDelta->deltaForward
             << ',' << odometryDelta->deltaLateral << ',' << odometryDelta->deltaTheta << ','
             << (lidarUpdated ? 1 : 0) << ',' << scanCount << ',' << scanMinAngle << ','
             << scanAngleIncrement << ',' << scanMaxRange << ',' << scanRangesSerialized << ','
             << scanPointsRobotSerialized << '\n';

    if (distanceGoal <= scenario.runtime.goalTolerance) {
      reachedGoal = true;
      break;
    }
  }

  *logFile << "# result=" << (reachedGoal && !failure ? "success" : "failure") << '\n';
  if (failure) {
    *logFile << "# error=" << failure->message << '\n';
    fmt::print(stderr, "Localization test error: {}\n", failure->message);
  }

  if (!reachedGoal) {
    fmt::print(stderr, "Localization test ended before reaching the goal.\n");
    return 1;
  }

  fmt::print("Localization test reached goal. Log: test/localization/logs/localization_test.log\n");
  return 0;
}
