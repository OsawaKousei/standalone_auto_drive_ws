#include "scenario_runtime.hpp"

#include "../features/control/controller_factory.hpp"
#include "../features/localization/localizer_factory.hpp"
#include "../features/planning/planner_factory.hpp"
#include "../features/simulation/simulation_factory.hpp"
#include "text_config.hpp"

#include <filesystem>
#include <fmt/core.h>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

namespace ad::scenario {

namespace {

constexpr auto kDefaultScenarioConfigPath = "defaults/scenario.toml";
constexpr auto kDefaultLocalizationConfigPathPattern = "defaults/localization/{}.toml";
constexpr auto kDefaultPlanningConfigPathPattern = "defaults/planning/{}.toml";
constexpr auto kDefaultControlConfigPathPattern = "defaults/control/{}.toml";
constexpr auto kDefaultSensorConfigPathPattern = "defaults/sensor/{}.toml";
constexpr auto kDefaultPhysicsConfigPathPattern = "defaults/physics/{}.toml";
constexpr auto kPointCoordinateStride = std::size_t{2};
constexpr auto kMinFootprintValueCount = std::size_t{6};

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

auto requiredDouble(const config::TextConfig &cfg, std::string_view section, std::string_view key)
    -> Result<double> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return config::parseDoubleValue(*raw);
}

auto requiredInt(const config::TextConfig &cfg, std::string_view section, std::string_view key)
    -> Result<int> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return config::parseIntValue(*raw);
}

auto parsePose(const config::TextConfig &cfg, std::string_view section) -> Result<types::Pose> {
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

auto parseAlgorithmSpec(const config::TextConfig &cfg, std::string_view section)
    -> Result<AlgorithmSpec> {
  const auto algorithm = requiredString(cfg, section, "algorithm");
  if (!algorithm) {
    return tl::make_unexpected(algorithm.error());
  }

  const auto configPath = optionalString(cfg, section, "config_path");
  if (!configPath) {
    return tl::make_unexpected(configPath.error());
  }
  return AlgorithmSpec{.algorithm = *algorithm, .configPath = *configPath};
}

auto parseFootprintVertices(const config::TextConfig &cfg) -> Result<types::Footprint> {
  const auto raw = requiredRaw(cfg, "robot.footprint", "vertices");
  if (!raw) {
    return tl::make_unexpected(raw.error());
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
  const auto odometryDeltaT = requiredDouble(cfg, "simulation.runtime", "odometry_delta_t");
  if (!odometryDeltaT) {
    return tl::make_unexpected(odometryDeltaT.error());
  }

  const auto lidarDeltaT = requiredDouble(cfg, "simulation.runtime", "lidar_delta_t");
  if (!lidarDeltaT) {
    return tl::make_unexpected(lidarDeltaT.error());
  }

  const auto renderDeltaT = requiredDouble(cfg, "simulation.runtime", "render_delta_t");
  if (!renderDeltaT) {
    return tl::make_unexpected(renderDeltaT.error());
  }

  const auto maxSteps = requiredInt(cfg, "simulation.runtime", "max_steps");
  if (!maxSteps) {
    return tl::make_unexpected(maxSteps.error());
  }

  const auto goalTolerance = requiredDouble(cfg, "simulation.runtime", "goal_tolerance");
  if (!goalTolerance) {
    return tl::make_unexpected(goalTolerance.error());
  }

  if (*odometryDeltaT <= 0.0 || *lidarDeltaT <= 0.0 || *renderDeltaT <= 0.0 ||
      *lidarDeltaT < *odometryDeltaT || *maxSteps <= 0 || *goalTolerance <= 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "simulation.runtime has invalid values."});
  }

  return RuntimeConfig{.odometryDeltaT = *odometryDeltaT,
                       .lidarDeltaT = *lidarDeltaT,
                       .renderDeltaT = *renderDeltaT,
                       .maxSteps = *maxSteps,
                       .goalTolerance = *goalTolerance};
}

[[nodiscard]] auto parseCollisionConfig(const config::TextConfig &cfg)
    -> Result<simulation::CollisionCheckConfig> {
  const auto translation = requiredDouble(cfg, "simulation.collision", "max_translation_step");
  if (!translation) {
    return tl::make_unexpected(translation.error());
  }

  const auto rotation = requiredDouble(cfg, "simulation.collision", "max_rotation_step");
  if (!rotation) {
    return tl::make_unexpected(rotation.error());
  }

  if (*translation <= 0.0 || *rotation <= 0.0) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "simulation.collision values must be positive."});
  }

  return simulation::CollisionCheckConfig{.maxTranslationStep = *translation,
                                          .maxRotationStep = *rotation};
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

[[nodiscard]] auto loadMergedConfig(std::string_view baseDir, std::string_view defaultPath,
                                    const std::optional<std::string> &overridePath)
    -> Result<config::TextConfig> {
  const auto defaultFilePath = resolvePath(baseDir, defaultPath);
  const auto defaultLoaded = config::loadTextConfig(defaultFilePath);
  if (!defaultLoaded) {
    return tl::make_unexpected(defaultLoaded.error());
  }

  auto merged = *defaultLoaded;
  if (!overridePath.has_value()) {
    return merged;
  }

  const auto overrideFilePath = resolvePath(baseDir, *overridePath);
  const auto overrideLoaded = config::loadTextConfig(overrideFilePath);
  if (!overrideLoaded) {
    return tl::make_unexpected(overrideLoaded.error());
  }

  merged.mergeFrom(*overrideLoaded);
  return merged;
}

[[nodiscard]] auto defaultAlgorithmConfigPath(std::string_view section, std::string_view algorithm)
    -> Result<std::string> {
  if (section == "localization") {
    return fmt::format(kDefaultLocalizationConfigPathPattern, algorithm);
  }
  if (section == "planning") {
    return fmt::format(kDefaultPlanningConfigPathPattern, algorithm);
  }
  if (section == "control") {
    return fmt::format(kDefaultControlConfigPathPattern, algorithm);
  }
  if (section == "lidar_sensor" || section == "odometry_sensor") {
    return fmt::format(kDefaultSensorConfigPathPattern, algorithm);
  }
  if (section == "physics") {
    return fmt::format(kDefaultPhysicsConfigPathPattern, algorithm);
  }
  return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                   .message = "Unsupported algorithm section for default config: " +
                                              std::string{section}});
}

} // namespace

auto resolvePath(const ScenarioConfig &scenario, std::string_view path) -> std::string {
  return resolvePath(scenario.baseDir, path);
}

auto loadScenario(std::string_view scenarioPath) -> Result<ScenarioConfig> {
  const auto scenarioPathFs = std::filesystem::path{std::string{scenarioPath}};
  const auto baseDir = scenarioPathFs.parent_path().empty() ? std::filesystem::path{"."}
                                                            : scenarioPathFs.parent_path();
  const auto baseDirNormalized = baseDir.lexically_normal().string();
  const auto scenarioFileName = scenarioPathFs.filename().string();

  const auto cfg = loadMergedConfig(baseDirNormalized, kDefaultScenarioConfigPath,
                                    std::optional<std::string>{scenarioFileName});
  if (!cfg) {
    return tl::make_unexpected(cfg.error());
  }

  const auto scenarioName = requiredString(*cfg, "scenario", "name");
  if (!scenarioName) {
    return tl::make_unexpected(scenarioName.error());
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

  const auto runtime = parseRuntimeConfig(*cfg);
  if (!runtime) {
    return tl::make_unexpected(runtime.error());
  }

  const auto collision = parseCollisionConfig(*cfg);
  if (!collision) {
    return tl::make_unexpected(collision.error());
  }

  const auto localizationSpec = parseAlgorithmSpec(*cfg, "localization");
  if (!localizationSpec) {
    return tl::make_unexpected(localizationSpec.error());
  }
  const auto planningSpec = parseAlgorithmSpec(*cfg, "planning");
  if (!planningSpec) {
    return tl::make_unexpected(planningSpec.error());
  }
  const auto controlSpec = parseAlgorithmSpec(*cfg, "control");
  if (!controlSpec) {
    return tl::make_unexpected(controlSpec.error());
  }
  const auto lidarSensorSpec = parseAlgorithmSpec(*cfg, "lidar_sensor");
  if (!lidarSensorSpec) {
    return tl::make_unexpected(lidarSensorSpec.error());
  }
  const auto odometrySensorSpec = parseAlgorithmSpec(*cfg, "odometry_sensor");
  if (!odometrySensorSpec) {
    return tl::make_unexpected(odometrySensorSpec.error());
  }
  const auto physicsSpec = parseAlgorithmSpec(*cfg, "physics");
  if (!physicsSpec) {
    return tl::make_unexpected(physicsSpec.error());
  }

  const auto localizationDefaultPath =
      defaultAlgorithmConfigPath("localization", localizationSpec->algorithm);
  if (!localizationDefaultPath) {
    return tl::make_unexpected(localizationDefaultPath.error());
  }
  const auto localizationDoc =
      loadMergedConfig(baseDirNormalized, *localizationDefaultPath, localizationSpec->configPath);
  if (!localizationDoc) {
    return tl::make_unexpected(localizationDoc.error());
  }

  const auto planningDefaultPath = defaultAlgorithmConfigPath("planning", planningSpec->algorithm);
  if (!planningDefaultPath) {
    return tl::make_unexpected(planningDefaultPath.error());
  }
  const auto planningDoc =
      loadMergedConfig(baseDirNormalized, *planningDefaultPath, planningSpec->configPath);
  if (!planningDoc) {
    return tl::make_unexpected(planningDoc.error());
  }

  const auto controlDefaultPath = defaultAlgorithmConfigPath("control", controlSpec->algorithm);
  if (!controlDefaultPath) {
    return tl::make_unexpected(controlDefaultPath.error());
  }
  const auto controlDoc =
      loadMergedConfig(baseDirNormalized, *controlDefaultPath, controlSpec->configPath);
  if (!controlDoc) {
    return tl::make_unexpected(controlDoc.error());
  }

  const auto lidarSensorDefaultPath =
      defaultAlgorithmConfigPath("lidar_sensor", lidarSensorSpec->algorithm);
  if (!lidarSensorDefaultPath) {
    return tl::make_unexpected(lidarSensorDefaultPath.error());
  }
  const auto lidarSensorDoc =
      loadMergedConfig(baseDirNormalized, *lidarSensorDefaultPath, lidarSensorSpec->configPath);
  if (!lidarSensorDoc) {
    return tl::make_unexpected(lidarSensorDoc.error());
  }

  const auto odometrySensorDefaultPath =
      defaultAlgorithmConfigPath("odometry_sensor", odometrySensorSpec->algorithm);
  if (!odometrySensorDefaultPath) {
    return tl::make_unexpected(odometrySensorDefaultPath.error());
  }
  const auto odometrySensorDoc = loadMergedConfig(baseDirNormalized, *odometrySensorDefaultPath,
                                                  odometrySensorSpec->configPath);
  if (!odometrySensorDoc) {
    return tl::make_unexpected(odometrySensorDoc.error());
  }

  const auto physicsDefaultPath = defaultAlgorithmConfigPath("physics", physicsSpec->algorithm);
  if (!physicsDefaultPath) {
    return tl::make_unexpected(physicsDefaultPath.error());
  }
  const auto physicsDoc =
      loadMergedConfig(baseDirNormalized, *physicsDefaultPath, physicsSpec->configPath);
  if (!physicsDoc) {
    return tl::make_unexpected(physicsDoc.error());
  }

  const auto initialCovariance = localization::parseInitialCovarianceFromConfig(
      std::optional<config::TextConfig>{*localizationDoc});
  if (!initialCovariance) {
    return tl::make_unexpected(initialCovariance.error());
  }

  return ScenarioConfig{.name = *scenarioName,
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
                        .algorithmConfigDocs = AlgorithmConfigDocs{
                            .localization = std::optional<config::TextConfig>{*localizationDoc},
                            .planning = std::optional<config::TextConfig>{*planningDoc},
                            .control = std::optional<config::TextConfig>{*controlDoc},
                            .lidarSensor = std::optional<config::TextConfig>{*lidarSensorDoc},
                            .odometrySensor = std::optional<config::TextConfig>{*odometrySensorDoc},
                            .physics = std::optional<config::TextConfig>{*physicsDoc}}};
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
