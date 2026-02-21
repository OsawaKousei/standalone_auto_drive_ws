#include "scenario_runtime.hpp"

#include "../features/control/controller_factory.hpp"
#include "../features/localization/localizer_factory.hpp"
#include "../features/planning/planner_factory.hpp"
#include "../features/simulation/simulation_factory.hpp"
#include "text_config.hpp"

#include <filesystem>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

namespace ad::scenario {

namespace {

constexpr auto kDefaultOdometryDeltaT = 0.02;
constexpr auto kDefaultLidarDeltaT = 0.2;
constexpr auto kDefaultRenderDeltaT = 0.1;
constexpr auto kDefaultGoalTolerance = 0.3;
constexpr auto kDefaultMaxSteps = 250;
constexpr auto kDefaultGoalX = 9.0;
constexpr auto kDefaultFootprintRearX = -0.2;
constexpr auto kDefaultFootprintFrontX = 0.3;
constexpr auto kDefaultFootprintHalfWidth = 0.1;
constexpr auto kPointCoordinateStride = std::size_t{2};
constexpr auto kMinFootprintValueCount = std::size_t{6};
constexpr auto kDefaultCollisionMaxTranslationStep = 0.05;
constexpr auto kDefaultCollisionMaxRotationStep = 0.05;

[[nodiscard]] auto makeDefaultStartPose() -> types::Pose {
  return types::Pose{.x = 1.0, .y = 1.0, .theta = 0.0};
}

[[nodiscard]] auto makeDefaultGoalPose() -> types::Pose {
  return types::Pose{.x = kDefaultGoalX, .y = 1.0, .theta = 0.0};
}

[[nodiscard]] auto makeDefaultFootprint() -> types::Footprint {
  return types::Footprint{{{kDefaultFootprintRearX, -kDefaultFootprintHalfWidth},
                           {kDefaultFootprintFrontX, -kDefaultFootprintHalfWidth},
                           {kDefaultFootprintFrontX, kDefaultFootprintHalfWidth},
                           {kDefaultFootprintRearX, kDefaultFootprintHalfWidth}}};
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
  xValue = xOverride->value_or(xValue);

  const auto yOverride = optionalDouble(cfg, section, "y");
  if (!yOverride) {
    return tl::make_unexpected(yOverride.error());
  }
  yValue = yOverride->value_or(yValue);

  const auto thetaOverride = optionalDouble(cfg, section, "theta");
  if (!thetaOverride) {
    return tl::make_unexpected(thetaOverride.error());
  }
  thetaValue = thetaOverride->value_or(thetaValue);

  return types::Pose{.x = xValue, .y = yValue, .theta = thetaValue};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto parseAlgorithmSpec(const config::TextConfig &cfg, std::string_view section,
                        std::string_view defaultAlgorithm) -> Result<AlgorithmSpec> {
  auto algorithm = std::string{defaultAlgorithm};
  const auto algorithmOverride = optionalString(cfg, section, "algorithm");
  if (!algorithmOverride) {
    return tl::make_unexpected(algorithmOverride.error());
  }
  algorithm = algorithmOverride->value_or(algorithm);

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
  if (values->size() < kMinFootprintValueCount || values->size() % kPointCoordinateStride != 0U) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "robot.footprint.vertices must contain N x 2 numeric values."});
  }

  auto vertices = std::vector<types::Point>{};
  vertices.reserve(values->size() / kPointCoordinateStride);
  const auto pairIndices =
      std::views::iota(std::size_t{0}, values->size() / kPointCoordinateStride);
  std::ranges::transform(
      pairIndices, std::back_inserter(vertices), [&](const std::size_t pairIndex) -> types::Point {
        const auto baseIndex = pairIndex * kPointCoordinateStride;
        return types::Point{.x = (*values)[baseIndex], .y = (*values)[baseIndex + 1U]};
      });
  return types::Footprint{std::move(vertices)};
}

[[nodiscard]] auto parseRuntimeConfig(const config::TextConfig &cfg) -> Result<RuntimeConfig> {
  auto odometryDeltaTValue = kDefaultOdometryDeltaT;
  auto lidarDeltaTValue = kDefaultLidarDeltaT;
  auto renderDeltaTValue = kDefaultRenderDeltaT;
  auto maxStepsValue = kDefaultMaxSteps;
  auto goalToleranceValue = kDefaultGoalTolerance;

  const auto odometryDeltaT = optionalDouble(cfg, "simulation.runtime", "odometry_delta_t");
  if (!odometryDeltaT) {
    return tl::make_unexpected(odometryDeltaT.error());
  }
  odometryDeltaTValue = odometryDeltaT->value_or(odometryDeltaTValue);

  const auto lidarDeltaT = optionalDouble(cfg, "simulation.runtime", "lidar_delta_t");
  if (!lidarDeltaT) {
    return tl::make_unexpected(lidarDeltaT.error());
  }
  lidarDeltaTValue = lidarDeltaT->value_or(lidarDeltaTValue);

  const auto renderDeltaT = optionalDouble(cfg, "simulation.runtime", "render_delta_t");
  if (!renderDeltaT) {
    return tl::make_unexpected(renderDeltaT.error());
  }
  renderDeltaTValue = renderDeltaT->value_or(renderDeltaTValue);

  const auto maxSteps = optionalInt(cfg, "simulation.runtime", "max_steps");
  if (!maxSteps) {
    return tl::make_unexpected(maxSteps.error());
  }
  maxStepsValue = maxSteps->value_or(maxStepsValue);

  const auto goalTolerance = optionalDouble(cfg, "simulation.runtime", "goal_tolerance");
  if (!goalTolerance) {
    return tl::make_unexpected(goalTolerance.error());
  }
  goalToleranceValue = goalTolerance->value_or(goalToleranceValue);

  if (odometryDeltaTValue <= 0.0 || lidarDeltaTValue <= 0.0 || renderDeltaTValue <= 0.0 ||
      lidarDeltaTValue < odometryDeltaTValue || maxStepsValue <= 0 || goalToleranceValue <= 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "simulation.runtime has invalid values."});
  }

  return RuntimeConfig{.odometryDeltaT = odometryDeltaTValue,
                       .lidarDeltaT = lidarDeltaTValue,
                       .renderDeltaT = renderDeltaTValue,
                       .maxSteps = maxStepsValue,
                       .goalTolerance = goalToleranceValue};
}

[[nodiscard]] auto parseCollisionConfig(const config::TextConfig &cfg)
    -> Result<simulation::CollisionCheckConfig> {
  auto maxTranslationStepValue = kDefaultCollisionMaxTranslationStep;
  auto maxRotationStepValue = kDefaultCollisionMaxRotationStep;

  const auto translation = optionalDouble(cfg, "simulation.collision", "max_translation_step");
  if (!translation) {
    return tl::make_unexpected(translation.error());
  }
  maxTranslationStepValue = translation->value_or(maxTranslationStepValue);

  const auto rotation = optionalDouble(cfg, "simulation.collision", "max_rotation_step");
  if (!rotation) {
    return tl::make_unexpected(rotation.error());
  }
  maxRotationStepValue = rotation->value_or(maxRotationStepValue);

  if (maxTranslationStepValue <= 0.0 || maxRotationStepValue <= 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "simulation.collision values must be positive."});
  }

  return simulation::CollisionCheckConfig{.maxTranslationStep = maxTranslationStepValue,
                                          .maxRotationStep = maxRotationStepValue};
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

[[nodiscard]] auto loadIfExists(std::string_view baseDir, const std::optional<std::string> &path)
    -> Result<std::optional<config::TextConfig>> {
  if (!path.has_value()) {
    return std::optional<config::TextConfig>{};
  }
  const auto filePath = resolvePath(baseDir, *path);
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
  const auto planningSpec = parseAlgorithmSpec(cfg, "planning", "astar");
  if (!planningSpec) {
    return tl::make_unexpected(planningSpec.error());
  }
  const auto controlSpec = parseAlgorithmSpec(cfg, "control", "pure_pursuit");
  if (!controlSpec) {
    return tl::make_unexpected(controlSpec.error());
  }
  const auto lidarSensorSpec = parseAlgorithmSpec(cfg, "lidar_sensor", "lidar");
  if (!lidarSensorSpec) {
    return tl::make_unexpected(lidarSensorSpec.error());
  }
  const auto odometrySensorSpec = parseAlgorithmSpec(cfg, "odometry_sensor", "odometry");
  if (!odometrySensorSpec) {
    return tl::make_unexpected(odometrySensorSpec.error());
  }
  const auto physicsSpec = parseAlgorithmSpec(cfg, "physics", "unicycle");
  if (!physicsSpec) {
    return tl::make_unexpected(physicsSpec.error());
  }

  const auto scenarioPathFs = std::filesystem::path{std::string{scenarioPath}};
  const auto baseDir = scenarioPathFs.parent_path().empty() ? std::filesystem::path{"."}
                                                            : scenarioPathFs.parent_path();
  const auto baseDirNormalized = baseDir.lexically_normal().string();
  const auto localizationDoc = loadIfExists(baseDirNormalized, localizationSpec->configPath);
  if (!localizationDoc) {
    return tl::make_unexpected(localizationDoc.error());
  }
  const auto planningDoc = loadIfExists(baseDirNormalized, planningSpec->configPath);
  if (!planningDoc) {
    return tl::make_unexpected(planningDoc.error());
  }
  const auto controlDoc = loadIfExists(baseDirNormalized, controlSpec->configPath);
  if (!controlDoc) {
    return tl::make_unexpected(controlDoc.error());
  }
  const auto lidarSensorDoc = loadIfExists(baseDirNormalized, lidarSensorSpec->configPath);
  if (!lidarSensorDoc) {
    return tl::make_unexpected(lidarSensorDoc.error());
  }
  const auto odometrySensorDoc = loadIfExists(baseDirNormalized, odometrySensorSpec->configPath);
  if (!odometrySensorDoc) {
    return tl::make_unexpected(odometrySensorDoc.error());
  }
  const auto physicsDoc = loadIfExists(baseDirNormalized, physicsSpec->configPath);
  if (!physicsDoc) {
    return tl::make_unexpected(physicsDoc.error());
  }

  const auto initialCovariance = localization::parseInitialCovarianceFromConfig(*localizationDoc);
  if (!initialCovariance) {
    return tl::make_unexpected(initialCovariance.error());
  }

  return ScenarioConfig{.name = scenarioName->value_or("scenario"),
                        .baseDir = baseDirNormalized,
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
                        .lidarSensor = *lidarSensorSpec,
                        .odometrySensor = *odometrySensorSpec,
                        .physics = *physicsSpec,
                        .algorithmConfigDocs =
                            AlgorithmConfigDocs{.localization = *localizationDoc,
                                                .planning = *planningDoc,
                                                .control = *controlDoc,
                                                .lidarSensor = *lidarSensorDoc,
                                                .odometrySensor = *odometrySensorDoc,
                                                .physics = *physicsDoc}};
}

auto createLocalizer(const ScenarioConfig &scenario, const types::MapData &map)
    -> Result<std::unique_ptr<localization::ILocalizer>> {
  return localization::createLocalizerFromConfig(scenario.localization.algorithm, map,
                                                 scenario.algorithmConfigDocs.localization);
}

auto createPlanner(const ScenarioConfig &scenario, const types::MapData &map,
                   const types::Footprint &footprint)
    -> Result<std::unique_ptr<planning::IPlanner>> {
  return planning::createPlannerFromConfig(scenario.planning.algorithm, map, footprint,
                                           scenario.algorithmConfigDocs.planning);
}

auto createController(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<control::IController>> {
  return control::createControllerFromConfig(scenario.control.algorithm,
                                             scenario.algorithmConfigDocs.control);
}

auto createLidarSensor(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::LidarSensor>> {
  return simulation::createLidarSensorFromConfig(scenario.lidarSensor.algorithm,
                                                 scenario.algorithmConfigDocs.lidarSensor);
}

auto createOdometrySensor(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::OdometrySensor>> {
  return simulation::createOdometrySensorFromConfig(scenario.odometrySensor.algorithm,
                                                    scenario.algorithmConfigDocs.odometrySensor);
}

auto createPhysics(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::IPhysicsModel>> {
  return simulation::createPhysicsFromConfig(scenario.physics.algorithm,
                                             scenario.algorithmConfigDocs.physics);
}

} // namespace ad::scenario
