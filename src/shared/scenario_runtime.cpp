#include "scenario_runtime.hpp"

#include "../features/control/controller_factory.hpp"
#include "../features/localization/localizer_factory.hpp"
#include "../features/planning/planner_factory.hpp"
#include "../features/simulation/simulation_factory.hpp"
#include "text_config.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ad::scenario {

namespace {

constexpr auto kDefaultDeltaT = 0.2;
constexpr auto kDefaultGoalTolerance = 0.3;
constexpr auto kDefaultFrameDelayMs = 80;
constexpr auto kDefaultMaxSteps = 250;
constexpr auto kDefaultScoreThreshold = 0.7;
constexpr auto kDefaultMinSpeedScale = 0.4;
constexpr auto kDefaultMaxAbsAngular = 2.5;

[[nodiscard]] auto makeDefaultStartPose() -> types::Pose {
  return types::Pose{.x = 1.0, .y = 1.0, .theta = 0.0};
}

[[nodiscard]] auto makeDefaultGoalPose() -> types::Pose {
  return types::Pose{.x = 9.0, .y = 1.0, .theta = 0.0};
}

[[nodiscard]] auto makeDefaultFootprint() -> types::Footprint {
  return types::Footprint{{{-0.2, -0.1}, {0.3, -0.1}, {0.3, 0.1}, {-0.2, 0.1}}};
}

[[nodiscard]] auto requiredRaw(const config::TextConfig &cfg, std::string_view section,
                               std::string_view key) -> Result<std::string_view> {
  const auto value = cfg.findRaw(section, key);
  if (!value) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Required config key is missing: " +
                                                std::string{section} + "." + std::string{key}});
  }
  return *value;
}

auto optionalString(const config::TextConfig &cfg, std::string_view section, std::string_view key)
    -> Result<std::optional<std::string>> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return std::optional<std::string>{};
  }
  const auto parsed = config::parseQuotedString(*raw);
  if (!parsed) {
    return tl::make_unexpected(parsed.error());
  }
  return std::optional<std::string>{*parsed};
}

auto requiredString(const config::TextConfig &cfg, std::string_view section, std::string_view key)
    -> Result<std::string> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return config::parseQuotedString(*raw);
}

auto optionalDouble(const config::TextConfig &cfg, std::string_view section, std::string_view key)
    -> Result<std::optional<double>> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return std::optional<double>{};
  }
  const auto parsed = config::parseDoubleValue(*raw);
  if (!parsed) {
    return tl::make_unexpected(parsed.error());
  }
  return std::optional<double>{*parsed};
}

auto optionalInt(const config::TextConfig &cfg, std::string_view section, std::string_view key)
    -> Result<std::optional<int>> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return std::optional<int>{};
  }
  const auto parsed = config::parseIntValue(*raw);
  if (!parsed) {
    return tl::make_unexpected(parsed.error());
  }
  return std::optional<int>{*parsed};
}

auto parsePose(const config::TextConfig &cfg, std::string_view section, const types::Pose &fallback)
    -> Result<types::Pose> {
  auto xValue = fallback.x;
  auto yValue = fallback.y;
  auto thetaValue = fallback.theta;

  const auto xOverride = optionalDouble(cfg, section, "x");
  if (!xOverride) {
    return tl::make_unexpected(xOverride.error());
  }
  if (xOverride->has_value()) {
    xValue = **xOverride;
  }

  const auto yOverride = optionalDouble(cfg, section, "y");
  if (!yOverride) {
    return tl::make_unexpected(yOverride.error());
  }
  if (yOverride->has_value()) {
    yValue = **yOverride;
  }

  const auto thetaOverride = optionalDouble(cfg, section, "theta");
  if (!thetaOverride) {
    return tl::make_unexpected(thetaOverride.error());
  }
  if (thetaOverride->has_value()) {
    thetaValue = **thetaOverride;
  }

  return types::Pose{.x = xValue, .y = yValue, .theta = thetaValue};
}

auto parseAlgorithmSpec(const config::TextConfig &cfg, std::string_view section,
                        std::string_view defaultAlgorithm) -> Result<AlgorithmSpec> {
  auto algorithm = std::string{defaultAlgorithm};
  const auto algorithmOverride = optionalString(cfg, section, "algorithm");
  if (!algorithmOverride) {
    return tl::make_unexpected(algorithmOverride.error());
  }
  if (algorithmOverride->has_value()) {
    algorithm = **algorithmOverride;
  }

  const auto configPath = optionalString(cfg, section, "config_path");
  if (!configPath) {
    return tl::make_unexpected(configPath.error());
  }
  return AlgorithmSpec{.algorithm = algorithm, .configPath = *configPath};
}

auto parseFootprintVertices(const config::TextConfig &cfg) -> Result<types::Footprint> {
  const auto raw = cfg.findRaw("robot.footprint", "vertices");
  if (!raw) {
    return makeDefaultFootprint();
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

[[nodiscard]] auto parseRuntimeConfig(const config::TextConfig &cfg) -> Result<RuntimeConfig> {
  auto deltaTValue = kDefaultDeltaT;
  auto maxStepsValue = kDefaultMaxSteps;
  auto goalToleranceValue = kDefaultGoalTolerance;
  auto frameDelayMsValue = kDefaultFrameDelayMs;
  auto scoreThresholdValue = kDefaultScoreThreshold;
  auto minSpeedScaleValue = kDefaultMinSpeedScale;
  auto maxAbsAngularValue = kDefaultMaxAbsAngular;

  const auto deltaT = optionalDouble(cfg, "simulation.runtime", "delta_t");
  if (!deltaT) {
    return tl::make_unexpected(deltaT.error());
  }
  if (deltaT->has_value()) {
    deltaTValue = **deltaT;
  }

  const auto maxSteps = optionalInt(cfg, "simulation.runtime", "max_steps");
  if (!maxSteps) {
    return tl::make_unexpected(maxSteps.error());
  }
  if (maxSteps->has_value()) {
    maxStepsValue = **maxSteps;
  }

  const auto goalTolerance = optionalDouble(cfg, "simulation.runtime", "goal_tolerance");
  if (!goalTolerance) {
    return tl::make_unexpected(goalTolerance.error());
  }
  if (goalTolerance->has_value()) {
    goalToleranceValue = **goalTolerance;
  }

  const auto frameDelay = optionalInt(cfg, "simulation.runtime", "frame_delay_ms");
  if (!frameDelay) {
    return tl::make_unexpected(frameDelay.error());
  }
  if (frameDelay->has_value()) {
    frameDelayMsValue = **frameDelay;
  }

  const auto scoreThreshold = optionalDouble(cfg, "simulation.runtime", "score_threshold");
  if (!scoreThreshold) {
    return tl::make_unexpected(scoreThreshold.error());
  }
  if (scoreThreshold->has_value()) {
    scoreThresholdValue = **scoreThreshold;
  }

  const auto minSpeedScale = optionalDouble(cfg, "simulation.runtime", "min_speed_scale");
  if (!minSpeedScale) {
    return tl::make_unexpected(minSpeedScale.error());
  }
  if (minSpeedScale->has_value()) {
    minSpeedScaleValue = **minSpeedScale;
  }

  const auto maxAbsAngular = optionalDouble(cfg, "simulation.runtime", "max_abs_angular");
  if (!maxAbsAngular) {
    return tl::make_unexpected(maxAbsAngular.error());
  }
  if (maxAbsAngular->has_value()) {
    maxAbsAngularValue = **maxAbsAngular;
  }

  if (deltaTValue <= 0.0 || maxStepsValue <= 0 || goalToleranceValue <= 0.0 ||
      frameDelayMsValue < 0 || scoreThresholdValue <= 0.0 || minSpeedScaleValue <= 0.0 ||
      maxAbsAngularValue <= 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "simulation.runtime has invalid values."});
  }

  return RuntimeConfig{.deltaT = deltaTValue,
                       .maxSteps = maxStepsValue,
                       .goalTolerance = goalToleranceValue,
                       .frameDelayMs = frameDelayMsValue,
                       .scoreThreshold = scoreThresholdValue,
                       .minSpeedScale = minSpeedScaleValue,
                       .maxAbsAngular = maxAbsAngularValue};
}

[[nodiscard]] auto parseCollisionConfig(const config::TextConfig &cfg)
    -> Result<simulation::CollisionCheckConfig> {
  auto maxTranslationStepValue = 0.05;
  auto maxRotationStepValue = 0.05;

  const auto translation = optionalDouble(cfg, "simulation.collision", "max_translation_step");
  if (!translation) {
    return tl::make_unexpected(translation.error());
  }
  if (translation->has_value()) {
    maxTranslationStepValue = **translation;
  }

  const auto rotation = optionalDouble(cfg, "simulation.collision", "max_rotation_step");
  if (!rotation) {
    return tl::make_unexpected(rotation.error());
  }
  if (rotation->has_value()) {
    maxRotationStepValue = **rotation;
  }

  if (maxTranslationStepValue <= 0.0 || maxRotationStepValue <= 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "simulation.collision values must be positive."});
  }

  return simulation::CollisionCheckConfig{.maxTranslationStep = maxTranslationStepValue,
                                          .maxRotationStep = maxRotationStepValue};
}

[[nodiscard]] auto resolvePath(std::string_view baseDir, std::string_view path) -> std::string {
  const auto candidate = std::filesystem::path{std::string{path}};
  if (candidate.is_absolute()) {
    return candidate.lexically_normal().string();
  }
  const auto resolved = std::filesystem::path{std::string{baseDir}} / candidate;
  return resolved.lexically_normal().string();
}

[[nodiscard]] auto loadIfExists(const ScenarioConfig &scenario,
                                const std::optional<std::string> &path)
    -> Result<std::optional<config::TextConfig>> {
  if (!path.has_value()) {
    return std::optional<config::TextConfig>{};
  }
  const auto filePath = resolvePath(scenario, *path);
  const auto loaded = config::loadTextConfig(filePath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  return std::optional<config::TextConfig>{*loaded};
}

} // namespace

auto resolvePath(const ScenarioConfig &scenario, std::string_view path) -> std::string {
  return resolvePath(scenario.baseDir, path);
}

auto loadScenario(std::string_view scenarioPath) -> Result<ScenarioConfig> {
  const auto configResult = config::loadTextConfig(scenarioPath);
  if (!configResult) {
    return tl::make_unexpected(configResult.error());
  }
  const auto &cfg = *configResult;

  const auto scenarioName = optionalString(cfg, "scenario", "name");
  if (!scenarioName) {
    return tl::make_unexpected(scenarioName.error());
  }

  const auto mapYamlPath = requiredString(cfg, "map", "yaml_path");
  if (!mapYamlPath) {
    return tl::make_unexpected(mapYamlPath.error());
  }

  const auto footprint = parseFootprintVertices(cfg);
  if (!footprint) {
    return tl::make_unexpected(footprint.error());
  }

  const auto start = parsePose(cfg, "robot.start", makeDefaultStartPose());
  if (!start) {
    return tl::make_unexpected(start.error());
  }
  const auto goal = parsePose(cfg, "robot.goal", makeDefaultGoalPose());
  if (!goal) {
    return tl::make_unexpected(goal.error());
  }

  const auto runtime = parseRuntimeConfig(cfg);
  if (!runtime) {
    return tl::make_unexpected(runtime.error());
  }

  const auto collision = parseCollisionConfig(cfg);
  if (!collision) {
    return tl::make_unexpected(collision.error());
  }

  const auto localizationSpec = parseAlgorithmSpec(cfg, "localization", "ekf");
  if (!localizationSpec) {
    return tl::make_unexpected(localizationSpec.error());
  }
  const auto scenarioPathFs = std::filesystem::path{std::string{scenarioPath}};
  const auto baseDir = scenarioPathFs.parent_path().empty() ? std::filesystem::path{"."}
                                                            : scenarioPathFs.parent_path();
  const auto provisionalScenario = ScenarioConfig{
      .name = scenarioName->value_or("scenario"),
      .baseDir = baseDir.lexically_normal().string(),
      .mapYamlPath = *mapYamlPath,
      .footprint = *footprint,
      .start = *start,
      .goal = *goal,
      .runtime = *runtime,
      .collision = *collision,
      .initialCovariance = localization::CovarianceMatrix::Identity(),
      .localization = *localizationSpec,
      .planning = AlgorithmSpec{.algorithm = "astar", .configPath = std::nullopt},
      .control = AlgorithmSpec{.algorithm = "pure_pursuit", .configPath = std::nullopt},
      .sensor = AlgorithmSpec{.algorithm = "lidar", .configPath = std::nullopt},
      .physics = AlgorithmSpec{.algorithm = "unicycle", .configPath = std::nullopt}};
  const auto localizationDoc = loadIfExists(provisionalScenario, localizationSpec->configPath);
  if (!localizationDoc) {
    return tl::make_unexpected(localizationDoc.error());
  }
  const auto initialCovariance = localization::parseInitialCovarianceFromConfig(*localizationDoc);
  if (!initialCovariance) {
    return tl::make_unexpected(initialCovariance.error());
  }

  const auto planningSpec = parseAlgorithmSpec(cfg, "planning", "astar");
  if (!planningSpec) {
    return tl::make_unexpected(planningSpec.error());
  }
  const auto controlSpec = parseAlgorithmSpec(cfg, "control", "pure_pursuit");
  if (!controlSpec) {
    return tl::make_unexpected(controlSpec.error());
  }
  const auto sensorSpec = parseAlgorithmSpec(cfg, "sensor", "lidar");
  if (!sensorSpec) {
    return tl::make_unexpected(sensorSpec.error());
  }
  const auto physicsSpec = parseAlgorithmSpec(cfg, "physics", "unicycle");
  if (!physicsSpec) {
    return tl::make_unexpected(physicsSpec.error());
  }

  return ScenarioConfig{.name = scenarioName->value_or("scenario"),
                        .baseDir = baseDir.lexically_normal().string(),
                        .mapYamlPath = *mapYamlPath,
                        .footprint = *footprint,
                        .start = *start,
                        .goal = *goal,
                        .runtime = *runtime,
                        .collision = *collision,
                        .initialCovariance = *initialCovariance,
                        .localization = *localizationSpec,
                        .planning = *planningSpec,
                        .control = *controlSpec,
                        .sensor = *sensorSpec,
                        .physics = *physicsSpec};
}

auto createLocalizer(const ScenarioConfig &scenario, const types::MapData &map)
    -> Result<std::unique_ptr<localization::ILocalizer>> {
  const auto loaded = loadIfExists(scenario, scenario.localization.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  return localization::createLocalizerFromConfig(scenario.localization.algorithm, map, *loaded);
}

auto createPlanner(const ScenarioConfig &scenario, const types::MapData &map,
                   const types::Footprint &footprint) -> Result<planning::PlannerComponents> {
  const auto loaded = loadIfExists(scenario, scenario.planning.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  return planning::createPlannerFromConfig(scenario.planning.algorithm, map, footprint, *loaded);
}

auto createController(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<control::IController>> {
  const auto loaded = loadIfExists(scenario, scenario.control.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  return control::createControllerFromConfig(scenario.control.algorithm, *loaded);
}

auto createSensor(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::ISensorModel>> {
  const auto loaded = loadIfExists(scenario, scenario.sensor.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  return simulation::createSensorFromConfig(scenario.sensor.algorithm, *loaded);
}

auto createPhysics(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::IPhysicsModel>> {
  const auto loaded = loadIfExists(scenario, scenario.physics.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  return simulation::createPhysicsFromConfig(scenario.physics.algorithm, *loaded);
}

} // namespace ad::scenario
