#include "scenario_runtime.hpp"

#include "../features/control/pure_pursuit.hpp"
#include "../features/localization/ekf_localizer.hpp"
#include "../features/localization/localization_config.hpp"
#include "../features/planning/astar_planner.hpp"
#include "../features/planning/dijkstra_planner.hpp"
#include "../features/simulation/lidar_sim.hpp"
#include "../features/simulation/unicycle_model.hpp"
#include "text_config.hpp"

#include <Eigen/Dense>
#include <cstddef>
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

[[nodiscard]] auto makeDefaultInitialCovariance() -> localization::CovarianceMatrix {
  localization::CovarianceMatrix covariance = localization::CovarianceMatrix::Zero();
  covariance(0, 0) = 0.5;
  covariance(1, 1) = 0.5;
  covariance(2, 2) = 0.2;
  return covariance;
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

[[nodiscard]] auto parseInitialCovariance(const config::TextConfig &cfg)
    -> Result<localization::CovarianceMatrix> {
  auto covariance = makeDefaultInitialCovariance();

  const auto xx = optionalDouble(cfg, "localization.initial_covariance", "xx");
  if (!xx) {
    return tl::make_unexpected(xx.error());
  }
  if (xx->has_value()) {
    covariance(0, 0) = **xx;
  }

  const auto yy = optionalDouble(cfg, "localization.initial_covariance", "yy");
  if (!yy) {
    return tl::make_unexpected(yy.error());
  }
  if (yy->has_value()) {
    covariance(1, 1) = **yy;
  }

  const auto tt = optionalDouble(cfg, "localization.initial_covariance", "tt");
  if (!tt) {
    return tl::make_unexpected(tt.error());
  }
  if (tt->has_value()) {
    covariance(2, 2) = **tt;
  }

  if (covariance(0, 0) <= 0.0 || covariance(1, 1) <= 0.0 || covariance(2, 2) <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "localization.initial_covariance must be positive."});
  }

  return covariance;
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

[[nodiscard]] auto parseEkfConfig(const ScenarioConfig &scenario)
    -> Result<localization::EkfLocalizerConfig> {
  auto configValue = localization::config::ekfLocalizerDefaultConfig();

  const auto loaded = loadIfExists(scenario, scenario.localization.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  if (!loaded->has_value()) {
    return configValue;
  }
  const auto &cfg = **loaded;

  const auto readInt = [&](std::string_view section, std::string_view key,
                           int current) -> Result<int> {
    const auto raw = cfg.findRaw(section, key);
    if (!raw) {
      return current;
    }
    return config::parseIntValue(*raw);
  };
  const auto readDouble = [&](std::string_view section, std::string_view key,
                              double current) -> Result<double> {
    const auto raw = cfg.findRaw(section, key);
    if (!raw) {
      return current;
    }
    return config::parseDoubleValue(*raw);
  };

  const auto thetaBins = readInt("hough", "theta_bins", configValue.hough.thetaBins);
  if (!thetaBins) {
    return tl::make_unexpected(thetaBins.error());
  }
  const auto rhoBins = readInt("hough", "rho_bins", configValue.hough.rhoBins);
  if (!rhoBins) {
    return tl::make_unexpected(rhoBins.error());
  }
  const auto minVotes = readInt("hough", "min_votes", configValue.hough.minVotes);
  if (!minVotes) {
    return tl::make_unexpected(minVotes.error());
  }
  const auto maxLines = readInt("hough", "max_lines", configValue.hough.maxLines);
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto inlierDistance =
      readDouble("hough", "inlier_distance", configValue.hough.inlierDistance);
  if (!inlierDistance) {
    return tl::make_unexpected(inlierDistance.error());
  }
  const auto minSegmentLength =
      readDouble("hough", "min_segment_length", configValue.hough.minSegmentLength);
  if (!minSegmentLength) {
    return tl::make_unexpected(minSegmentLength.error());
  }
  const auto mergeRho = readDouble("hough", "merge_rho", configValue.hough.mergeRho);
  if (!mergeRho) {
    return tl::make_unexpected(mergeRho.error());
  }
  const auto mergeTheta = readDouble("hough", "merge_theta", configValue.hough.mergeTheta);
  if (!mergeTheta) {
    return tl::make_unexpected(mergeTheta.error());
  }

  const auto processNoiseTranslation =
      readDouble("ekf", "process_noise_translation", configValue.ekf.processNoiseTranslation);
  if (!processNoiseTranslation) {
    return tl::make_unexpected(processNoiseTranslation.error());
  }
  const auto processNoiseRotation =
      readDouble("ekf", "process_noise_rotation", configValue.ekf.processNoiseRotation);
  if (!processNoiseRotation) {
    return tl::make_unexpected(processNoiseRotation.error());
  }
  const auto measurementNoiseRange =
      readDouble("ekf", "measurement_noise_range", configValue.ekf.measurementNoiseRange);
  if (!measurementNoiseRange) {
    return tl::make_unexpected(measurementNoiseRange.error());
  }
  const auto measurementNoiseAngle =
      readDouble("ekf", "measurement_noise_angle", configValue.ekf.measurementNoiseAngle);
  if (!measurementNoiseAngle) {
    return tl::make_unexpected(measurementNoiseAngle.error());
  }

  const auto maxAssociationDistance =
      readDouble("association", "max_association_distance", configValue.maxAssociationDistance);
  if (!maxAssociationDistance) {
    return tl::make_unexpected(maxAssociationDistance.error());
  }
  const auto segmentMargin = readDouble("association", "segment_margin", configValue.segmentMargin);
  if (!segmentMargin) {
    return tl::make_unexpected(segmentMargin.error());
  }
  const auto gateThreshold = readDouble("association", "gate_threshold", configValue.gateThreshold);
  if (!gateThreshold) {
    return tl::make_unexpected(gateThreshold.error());
  }
  const auto minObservations =
      readInt("association", "min_observations", static_cast<int>(configValue.minObservations));
  if (!minObservations) {
    return tl::make_unexpected(minObservations.error());
  }

  return localization::EkfLocalizerConfig{
      .hough = localization::HoughConfig{.thetaBins = *thetaBins,
                                         .rhoBins = *rhoBins,
                                         .minVotes = *minVotes,
                                         .maxLines = *maxLines,
                                         .inlierDistance = *inlierDistance,
                                         .minSegmentLength = *minSegmentLength,
                                         .mergeRho = *mergeRho,
                                         .mergeTheta = *mergeTheta},
      .ekf = localization::EkfConfig{.processNoiseTranslation = *processNoiseTranslation,
                                     .processNoiseRotation = *processNoiseRotation,
                                     .measurementNoiseRange = *measurementNoiseRange,
                                     .measurementNoiseAngle = *measurementNoiseAngle},
      .maxAssociationDistance = *maxAssociationDistance,
      .segmentMargin = *segmentMargin,
      .gateThreshold = *gateThreshold,
      .minObservations = static_cast<std::size_t>(*minObservations)};
}

[[nodiscard]] auto parsePurePursuitConfig(const ScenarioConfig &scenario)
    -> Result<control::PurePursuitConfig> {
  auto lookaheadDistanceValue = control::config::kDefaultLookaheadDistance;
  auto desiredLinearVelocityValue = control::config::kDefaultDesiredLinearVelocity;
  const auto loaded = loadIfExists(scenario, scenario.control.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  if (!loaded->has_value()) {
    return control::config::purePursuitDefaultConfig();
  }

  const auto &cfg = **loaded;
  const auto lookaheadRaw = cfg.findRaw("", "lookahead_distance");
  if (lookaheadRaw) {
    const auto parsed = config::parseDoubleValue(*lookaheadRaw);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    lookaheadDistanceValue = *parsed;
  }

  const auto velocityRaw = cfg.findRaw("", "desired_linear_velocity");
  if (velocityRaw) {
    const auto parsed = config::parseDoubleValue(*velocityRaw);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    desiredLinearVelocityValue = *parsed;
  }

  return control::PurePursuitConfig{.lookaheadDistance = lookaheadDistanceValue,
                                    .desiredLinearVelocity = desiredLinearVelocityValue};
}

[[nodiscard]] auto parseLidarConfig(const ScenarioConfig &scenario)
    -> Result<simulation::LidarSimConfig> {
  auto rayCountValue = simulation::config::kDefaultRayCount;
  auto minAngleValue = simulation::config::kDefaultMinAngle;
  auto maxAngleValue = simulation::config::kDefaultMaxAngle;
  auto maxRangeValue = simulation::config::kDefaultMaxRange;
  auto rangeStepValue = simulation::config::kDefaultRangeStep;
  const auto loaded = loadIfExists(scenario, scenario.sensor.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  if (!loaded->has_value()) {
    return simulation::config::lidarDefaultConfig();
  }

  const auto &cfg = **loaded;
  const auto rayCount = cfg.findRaw("", "ray_count");
  if (rayCount) {
    const auto parsed = config::parseIntValue(*rayCount);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    rayCountValue = *parsed;
  }

  const auto minAngle = cfg.findRaw("", "min_angle");
  if (minAngle) {
    const auto parsed = config::parseDoubleValue(*minAngle);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    minAngleValue = *parsed;
  }

  const auto maxAngle = cfg.findRaw("", "max_angle");
  if (maxAngle) {
    const auto parsed = config::parseDoubleValue(*maxAngle);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxAngleValue = *parsed;
  }

  const auto maxRange = cfg.findRaw("", "max_range");
  if (maxRange) {
    const auto parsed = config::parseDoubleValue(*maxRange);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxRangeValue = *parsed;
  }

  const auto rangeStep = cfg.findRaw("", "range_step");
  if (rangeStep) {
    const auto parsed = config::parseDoubleValue(*rangeStep);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    rangeStepValue = *parsed;
  }

  return simulation::LidarSimConfig{.rayCount = rayCountValue,
                                    .minAngle = minAngleValue,
                                    .maxAngle = maxAngleValue,
                                    .maxRange = maxRangeValue,
                                    .rangeStep = rangeStepValue};
}

[[nodiscard]] auto parseUnicycleConfig(const ScenarioConfig &scenario)
    -> Result<simulation::UnicycleModelConfig> {
  auto maxLinearSpeedValue = simulation::config::kDefaultMaxLinearSpeed;
  auto maxAngularSpeedValue = simulation::config::kDefaultMaxAngularSpeed;
  const auto loaded = loadIfExists(scenario, scenario.physics.configPath);
  if (!loaded) {
    return tl::make_unexpected(loaded.error());
  }
  if (!loaded->has_value()) {
    return simulation::config::unicycleDefaultConfig();
  }

  const auto &cfg = **loaded;
  const auto linear = cfg.findRaw("", "max_linear_speed");
  if (linear) {
    const auto parsed = config::parseDoubleValue(*linear);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxLinearSpeedValue = *parsed;
  }

  const auto angular = cfg.findRaw("", "max_angular_speed");
  if (angular) {
    const auto parsed = config::parseDoubleValue(*angular);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    maxAngularSpeedValue = *parsed;
  }

  return simulation::UnicycleModelConfig{.maxLinearSpeed = maxLinearSpeedValue,
                                         .maxAngularSpeed = maxAngularSpeedValue};
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

  const auto initialCovariance = parseInitialCovariance(cfg);
  if (!initialCovariance) {
    return tl::make_unexpected(initialCovariance.error());
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
  const auto sensorSpec = parseAlgorithmSpec(cfg, "sensor", "lidar");
  if (!sensorSpec) {
    return tl::make_unexpected(sensorSpec.error());
  }
  const auto physicsSpec = parseAlgorithmSpec(cfg, "physics", "unicycle");
  if (!physicsSpec) {
    return tl::make_unexpected(physicsSpec.error());
  }

  const auto scenarioPathFs = std::filesystem::path{std::string{scenarioPath}};
  const auto baseDir = scenarioPathFs.parent_path().empty() ? std::filesystem::path{"."}
                                                            : scenarioPathFs.parent_path();

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
  if (scenario.localization.algorithm != "ekf") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported localization algorithm: " + scenario.localization.algorithm});
  }

  const auto configValue = parseEkfConfig(scenario);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }
  auto localizer = localization::EkfLocalizer::create(map, *configValue);
  if (!localizer) {
    return tl::make_unexpected(localizer.error());
  }

  auto derived = std::move(*localizer);
  std::unique_ptr<localization::ILocalizer> base{derived.release()};
  return base;
}

auto createPlanner(const ScenarioConfig &scenario,
                   const planning::ICollisionChecker &collisionChecker)
    -> Result<std::unique_ptr<planning::IPlanner>> {
  if (scenario.planning.algorithm == "astar") {
    return std::unique_ptr<planning::IPlanner>{new planning::AStarPlanner{collisionChecker}};
  }
  if (scenario.planning.algorithm == "dijkstra") {
    return std::unique_ptr<planning::IPlanner>{new planning::DijkstraPlanner{collisionChecker}};
  }
  return tl::make_unexpected(
      Error{.code = ErrorCode::InvalidInput,
            .message = "Unsupported planning algorithm: " + scenario.planning.algorithm});
}

auto createController(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<control::IController>> {
  if (scenario.control.algorithm != "pure_pursuit") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported control algorithm: " + scenario.control.algorithm});
  }

  const auto configValue = parsePurePursuitConfig(scenario);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::unique_ptr<control::IController>{new control::PurePursuitController{*configValue}};
}

auto createSensor(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::ISensorModel>> {
  if (scenario.sensor.algorithm != "lidar") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported sensor algorithm: " + scenario.sensor.algorithm});
  }

  const auto configValue = parseLidarConfig(scenario);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::unique_ptr<simulation::ISensorModel>{new simulation::LidarSim{*configValue}};
}

auto createPhysics(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::IPhysicsModel>> {
  if (scenario.physics.algorithm != "unicycle") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported physics algorithm: " + scenario.physics.algorithm});
  }

  const auto configValue = parseUnicycleConfig(scenario);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::unique_ptr<simulation::IPhysicsModel>{new simulation::UnicycleModel{*configValue}};
}

} // namespace ad::scenario
