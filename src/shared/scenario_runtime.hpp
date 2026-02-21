#pragma once

#include "../features/control/i_controller.hpp"
#include "../features/localization/i_localizer.hpp"
#include "../features/planning/i_planner.hpp"
#include "../features/planning/planner_factory.hpp"
#include "../features/simulation/collision_checker/collision_checker.hpp"
#include "../features/simulation/i_physics.hpp"
#include "../features/simulation/sensor/lidar_sensor.hpp"
#include "../features/simulation/sensor/odometry_sensor.hpp"
#include "result.hpp"
#include "text_config.hpp"
#include "types.hpp"

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace ad::scenario {

struct AlgorithmSpec {
  const std::string algorithm;
  const std::optional<std::string> configPath;
};

struct RuntimeConfig {
  const double stepSeconds;
  const double lidarDeltaT;
  const double renderDeltaT;
  const int maxSteps;
  const double goalTolerance;
};

struct AlgorithmConfigDocs {
  const std::optional<config::TextConfig> localization;
  const std::optional<config::TextConfig> planning;
  const std::optional<config::TextConfig> control;
  const std::optional<config::TextConfig> lidarSensor;
  const std::optional<config::TextConfig> odometrySensor;
  const std::optional<config::TextConfig> physics;
};

struct ScenarioConfig {
  const std::string name;
  const std::string baseDir;
  const std::string mapYamlPath;
  const types::Footprint footprint;
  const types::Pose start;
  const types::Pose goal;
  const RuntimeConfig runtime;
  const simulation::CollisionCheckConfig collision;
  const localization::CovarianceMatrix initialCovariance;
  const AlgorithmSpec localization;
  const AlgorithmSpec planning;
  const AlgorithmSpec control;
  const AlgorithmSpec lidarSensor;
  const AlgorithmSpec odometrySensor;
  const AlgorithmSpec physics;
  const AlgorithmConfigDocs algorithmConfigDocs;
};

[[nodiscard]] auto loadScenario(std::string_view scenarioPath) -> Result<ScenarioConfig>;
[[nodiscard]] auto createLocalizer(const ScenarioConfig &scenario, const types::MapData &map)
    -> Result<std::unique_ptr<localization::ILocalizer>>;
[[nodiscard]] auto createPlanner(const ScenarioConfig &scenario, const types::MapData &map,
                                 const types::Footprint &footprint)
    -> Result<std::unique_ptr<planning::IPlanner>>;
[[nodiscard]] auto createController(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<control::IController>>;
[[nodiscard]] auto createLidarSensor(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::LidarSensor>>;
[[nodiscard]] auto createOdometrySensor(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::OdometrySensor>>;
[[nodiscard]] auto createPhysics(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::IPhysicsModel>>;
[[nodiscard]] auto resolvePath(const ScenarioConfig &scenario, std::string_view path)
    -> std::string;

} // namespace ad::scenario
