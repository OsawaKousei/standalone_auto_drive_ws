#pragma once

#include "../features/control/i_controller.hpp"
#include "../features/localization/i_localizer.hpp"
#include "../features/planning/i_planner.hpp"
#include "../features/planning/planner_factory.hpp"
#include "../features/simulation/collision_checker.hpp"
#include "../features/simulation/i_physics.hpp"
#include "../features/simulation/i_sensor.hpp"
#include "result.hpp"
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
  const double deltaT;
  const int maxSteps;
  const double goalTolerance;
  const int frameDelayMs;
  const double scoreThreshold;
  const double minSpeedScale;
  const double maxAbsAngular;
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
  const AlgorithmSpec sensor;
  const AlgorithmSpec physics;
};

[[nodiscard]] auto loadScenario(std::string_view scenarioPath) -> Result<ScenarioConfig>;
[[nodiscard]] auto createLocalizer(const ScenarioConfig &scenario, const types::MapData &map)
    -> Result<std::unique_ptr<localization::ILocalizer>>;
[[nodiscard]] auto createPlanner(const ScenarioConfig &scenario, const types::MapData &map,
                                 const types::Footprint &footprint)
    -> Result<std::unique_ptr<planning::IPlanner>>;
[[nodiscard]] auto createController(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<control::IController>>;
[[nodiscard]] auto createSensor(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::ISensorModel>>;
[[nodiscard]] auto createPhysics(const ScenarioConfig &scenario)
    -> Result<std::unique_ptr<simulation::IPhysicsModel>>;
[[nodiscard]] auto resolvePath(const ScenarioConfig &scenario, std::string_view path)
    -> std::string;

} // namespace ad::scenario
