#include "features/planning/planner_factory.hpp"
#include "features/planning/planner_utils.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/text_config.hpp"
#include "shared/types.hpp"

#include <algorithm>
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
#include <utility>
#include <vector>

namespace ad::planning_test {

constexpr auto kNumericEpsilon = 1.0e-12;
constexpr auto kMetricPrecision = 8;
constexpr auto kMinFootprintVertexValueCount = std::size_t{6};
constexpr auto kStartGoalMarkerSize = 26.0;

struct AlgorithmSpec {
  std::string algorithm;
  std::string configPath;
};

struct TestScenarioConfig {
  std::string name;
  std::string baseDir;
  std::string mapYamlPath;
  types::Footprint footprint;
  types::Pose start;
  types::Pose goal;
  AlgorithmSpec planning;
};

struct ProgramOptions {
  std::string scenarioPath;
};

struct PlannerMetrics {
  double pathLength;
  double maxCurvature;
  double maxCurvatureRate;
  double averageClearance;
  double minimumClearance;
  double dijkstraOptimalityRatio;
};

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] auto resolvePath(std::string_view baseDir, std::string_view path) -> std::string {
  const auto candidate = std::filesystem::path{std::string{path}};
  if (candidate.is_absolute()) {
    return candidate.lexically_normal().string();
  }
  return (std::filesystem::path{std::string{baseDir}} / candidate).lexically_normal().string();
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

  if (values->size() < kMinFootprintVertexValueCount || values->size() % 2U != 0U) {
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

  const auto planning = parseAlgorithmSpec(*cfg, "planning");
  if (!planning) {
    return tl::make_unexpected(planning.error());
  }

  return TestScenarioConfig{.name = *name,
                            .baseDir = baseDirNormalized,
                            .mapYamlPath = *mapYamlPath,
                            .footprint = *footprint,
                            .start = *start,
                            .goal = *goal,
                            .planning = *planning};
}

[[nodiscard]] auto loadAlgorithmConfig(std::string_view baseDir, std::string_view configPath)
    -> Result<config::TextConfig> {
  return config::loadTextConfig(resolvePath(baseDir, configPath));
}

[[nodiscard]] auto parseProgramOptions(std::span<char *> arguments) -> Result<ProgramOptions> {
  auto scenarioPath = std::string{"test/planning/configs/planning.toml"};

  for (std::size_t index = 1; index < arguments.size(); ++index) {
    const auto argument = std::string_view{arguments[index]};
    if (argument == "-h" || argument == "--help") {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Usage: planning_test_app [scenario.toml]"});
    }
    if (!argument.empty() && argument.front() == '-') {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Unknown option: " + std::string{argument}});
    }
    scenarioPath = std::string{argument};
  }

  return ProgramOptions{.scenarioPath = scenarioPath};
}

[[nodiscard]] auto computePathLength(std::span<const types::Point> path) -> double {
  if (path.size() < 2U) {
    return 0.0;
  }

  auto length = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    length += std::hypot(path[index].x - path[index - 1U].x, path[index].y - path[index - 1U].y);
  }
  return length;
}

[[nodiscard]] auto computeSignedCurvature(const types::Point &previous, const types::Point &current,
                                          const types::Point &next) -> double {
  const auto segmentA_dx = current.x - previous.x;
  const auto segmentA_dy = current.y - previous.y;
  const auto segmentB_dx = next.x - current.x;
  const auto segmentB_dy = next.y - current.y;
  const auto segmentC_dx = next.x - previous.x;
  const auto segmentC_dy = next.y - previous.y;

  const auto segmentA = std::hypot(segmentA_dx, segmentA_dy);
  const auto segmentB = std::hypot(segmentB_dx, segmentB_dy);
  const auto segmentC = std::hypot(segmentC_dx, segmentC_dy);
  if (segmentA <= kNumericEpsilon || segmentB <= kNumericEpsilon || segmentC <= kNumericEpsilon) {
    return 0.0;
  }

  const auto doubledArea = (segmentA_dx * segmentC_dy) - (segmentA_dy * segmentC_dx);
  return doubledArea / (segmentA * segmentB * segmentC);
}

[[nodiscard]] auto computeMaxCurvature(std::span<const types::Point> path) -> double {
  if (path.size() < 3U) {
    return 0.0;
  }

  auto maxCurvature = 0.0;
  for (std::size_t index = 1; index + 1U < path.size(); ++index) {
    maxCurvature =
        std::max(maxCurvature,
                 std::abs(computeSignedCurvature(path[index - 1U], path[index], path[index + 1U])));
  }
  return maxCurvature;
}

[[nodiscard]] auto computeMaxCurvatureRate(std::span<const types::Point> path) -> double {
  if (path.size() < 4U) {
    return 0.0;
  }

  auto curvatures = std::vector<double>{};
  auto arcAtCurvature = std::vector<double>{};
  curvatures.reserve(path.size() - 2U);
  arcAtCurvature.reserve(path.size() - 2U);

  auto cumulativeArc = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    cumulativeArc +=
        std::hypot(path[index].x - path[index - 1U].x, path[index].y - path[index - 1U].y);
    if (index + 1U < path.size()) {
      curvatures.push_back(computeSignedCurvature(path[index - 1U], path[index], path[index + 1U]));
      arcAtCurvature.push_back(cumulativeArc);
    }
  }

  auto maxRate = 0.0;
  for (std::size_t index = 1; index < curvatures.size(); ++index) {
    const auto deltaArc = arcAtCurvature[index] - arcAtCurvature[index - 1U];
    if (deltaArc <= kNumericEpsilon) {
      continue;
    }
    const auto rate = std::abs((curvatures[index] - curvatures[index - 1U]) / deltaArc);
    maxRate = std::max(maxRate, rate);
  }
  return maxRate;
}

[[nodiscard]] auto estimateFootprintRadius(const types::Footprint &footprint) -> double {
  if (footprint.vertices.empty()) {
    return 0.0;
  }

  auto radius = 0.0;
  for (const auto &vertex : footprint.vertices) {
    radius = std::max(radius, std::hypot(vertex.x, vertex.y));
  }
  return radius;
}

[[nodiscard]] auto buildObstacleCellCenters(const types::MapData &map)
    -> std::vector<types::Point> {
  auto cells = std::vector<types::Point>{};
  cells.reserve(map.grid.size() / 4U);
  for (std::size_t index = 0; index < map.grid.size(); ++index) {
    if (map.grid[index] <= 0) {
      continue;
    }
    const auto coord = planning::utils::toCoord(map, index);
    cells.push_back(planning::utils::cellCenter(map, coord));
  }
  return cells;
}

[[nodiscard]] auto clearanceToObstacles(const types::Point &point,
                                        std::span<const types::Point> obstacles,
                                        double obstacleHalfDiagonal, double footprintRadius)
    -> double {
  if (obstacles.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  auto nearestCenterDistance = std::numeric_limits<double>::infinity();
  for (const auto &obstacle : obstacles) {
    nearestCenterDistance =
        std::min(nearestCenterDistance, std::hypot(point.x - obstacle.x, point.y - obstacle.y));
  }

  return std::max(0.0, nearestCenterDistance - obstacleHalfDiagonal - footprintRadius);
}

[[nodiscard]] auto computeClearanceMetrics(std::span<const types::Point> path,
                                           const types::MapData &map,
                                           const types::Footprint &footprint)
    -> std::pair<double, double> {
  if (path.empty()) {
    return {0.0, 0.0};
  }

  const auto obstacles = buildObstacleCellCenters(map);
  if (obstacles.empty()) {
    return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }

  const auto footprintRadius = estimateFootprintRadius(footprint);
  const auto obstacleHalfDiagonal = 0.5 * std::numbers::sqrt2 * map.resolution;

  auto sum = 0.0;
  auto minClearance = std::numeric_limits<double>::infinity();
  for (const auto &point : path) {
    const auto clearance =
        clearanceToObstacles(point, obstacles, obstacleHalfDiagonal, footprintRadius);
    sum += clearance;
    minClearance = std::min(minClearance, clearance);
  }

  return {sum / static_cast<double>(path.size()), minClearance};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] auto computeMetrics(std::span<const types::Point> path,
                                  std::span<const types::Point> dijkstraPath,
                                  const types::MapData &map, const types::Footprint &footprint)
    -> Result<PlannerMetrics> {
  const auto pathLength = computePathLength(path);
  const auto dijkstraPathLength = computePathLength(dijkstraPath);
  if (dijkstraPathLength <= kNumericEpsilon) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Dijkstra path length is zero."});
  }

  const auto [averageClearance, minimumClearance] = computeClearanceMetrics(path, map, footprint);

  return PlannerMetrics{.pathLength = pathLength,
                        .maxCurvature = computeMaxCurvature(path),
                        .maxCurvatureRate = computeMaxCurvatureRate(path),
                        .averageClearance = averageClearance,
                        .minimumClearance = minimumClearance,
                        .dijkstraOptimalityRatio = pathLength / dijkstraPathLength};
}

[[nodiscard]] auto saveArtifacts(const TestScenarioConfig &scenario,
                                 std::span<const types::Point> path, const PlannerMetrics &metrics,
                                 const types::MapData &map) -> Status {
  std::error_code fsError;
  std::filesystem::create_directories("test/planning/logs", fsError);
  if (fsError) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Failed to create planning logs directory: " + fsError.message()});
  }

  auto visualizer = visualization::Visualizer{};
  const auto preparedMap = visualization::Visualizer::prepareMap(map);
  if (!preparedMap) {
    return tl::make_unexpected(preparedMap.error());
  }

  const auto frameStatus = visualizer.renderFrame(*preparedMap);
  if (!frameStatus) {
    return frameStatus;
  }

  const auto pathStatus = visualizer.renderPath(path, preparedMap->geometry, "b-");
  if (!pathStatus) {
    return pathStatus;
  }

  const auto startMarkerStatus =
      visualizer.renderMarker(types::Point{.x = scenario.start.x, .y = scenario.start.y},
                              preparedMap->geometry, kStartGoalMarkerSize, "green");
  if (!startMarkerStatus) {
    return startMarkerStatus;
  }

  const auto goalMarkerStatus =
      visualizer.renderMarker(types::Point{.x = scenario.goal.x, .y = scenario.goal.y},
                              preparedMap->geometry, kStartGoalMarkerSize, "red");
  if (!goalMarkerStatus) {
    return goalMarkerStatus;
  }

  const auto imageStatus = visualizer.saveFigure("test/planning/logs/planning_result.png");
  if (!imageStatus) {
    return imageStatus;
  }

  auto metricsFile = std::ofstream{"test/planning/logs/metrics.json"};
  if (!metricsFile.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to open metrics.json"});
  }

  metricsFile << std::fixed << std::setprecision(kMetricPrecision);
  metricsFile << "{\n";
  metricsFile << "  \"path_length\": " << metrics.pathLength << ",\n";
  metricsFile << "  \"max_curvature\": " << metrics.maxCurvature << ",\n";
  metricsFile << "  \"max_curvature_rate\": " << metrics.maxCurvatureRate << ",\n";
  metricsFile << "  \"average_clearance\": " << metrics.averageClearance << ",\n";
  metricsFile << "  \"minimum_clearance\": " << metrics.minimumClearance << ",\n";
  metricsFile << "  \"dijkstra_optimality_ratio\": " << metrics.dijkstraOptimalityRatio << "\n";
  metricsFile << "}\n";

  auto logFile = std::ofstream{"test/planning/logs/planning_test.log"};
  if (!logFile.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to open planning_test.log"});
  }

  logFile << std::fixed << std::setprecision(kMetricPrecision);
  logFile << "# planning_test log\n";
  logFile << "# scenario_name=" << scenario.name << "\n";
  logFile << "# output_png=test/planning/logs/planning_result.png\n";
  logFile << "path_length=" << metrics.pathLength << "\n";
  logFile << "max_curvature=" << metrics.maxCurvature << "\n";
  logFile << "max_curvature_rate=" << metrics.maxCurvatureRate << "\n";
  logFile << "average_clearance=" << metrics.averageClearance << "\n";
  logFile << "minimum_clearance=" << metrics.minimumClearance << "\n";
  logFile << "dijkstra_optimality_ratio=" << metrics.dijkstraOptimalityRatio << "\n";
  return {};
}

} // namespace ad::planning_test

auto main(int argc, char **argv) -> int {
  const auto arguments = std::span<char *>{argv, static_cast<std::size_t>(argc)};
  const auto optionsResult = ad::planning_test::parseProgramOptions(arguments);
  if (!optionsResult) {
    fmt::print(stderr, "Argument error: {}\n", optionsResult.error().message);
    return 1;
  }

  const auto scenarioResult = ad::planning_test::loadScenario(optionsResult->scenarioPath);
  if (!scenarioResult) {
    fmt::print(stderr, "Scenario load error: {}\n", scenarioResult.error().message);
    return 1;
  }
  const auto &scenario = *scenarioResult;

  const auto mapPath = ad::planning_test::resolvePath(scenario.baseDir, scenario.mapYamlPath);
  const auto mapResult = ad::loadMapFromYaml(mapPath);
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }
  const auto &map = *mapResult;

  const auto planningConfig =
      ad::planning_test::loadAlgorithmConfig(scenario.baseDir, scenario.planning.configPath);
  if (!planningConfig) {
    fmt::print(stderr, "Planning config load error: {}\n", planningConfig.error().message);
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
      plannerResult.value()->plan(map, scenario.start, scenario.goal, scenario.footprint);
  if (!pathResult) {
    fmt::print(stderr, "Planning error: {}\n", pathResult.error().message);
    return 1;
  }

  auto dijkstraPlannerResult = ad::planning::createPlannerFromConfig(
      "dijkstra", map, scenario.footprint, std::optional<ad::config::TextConfig>{*planningConfig});
  if (!dijkstraPlannerResult) {
    fmt::print(stderr, "Dijkstra planner create error: {}\n",
               dijkstraPlannerResult.error().message);
    return 1;
  }

  const auto dijkstraPathResult =
      dijkstraPlannerResult.value()->plan(map, scenario.start, scenario.goal, scenario.footprint);
  if (!dijkstraPathResult) {
    fmt::print(stderr, "Dijkstra planning error: {}\n", dijkstraPathResult.error().message);
    return 1;
  }

  const auto metrics = ad::planning_test::computeMetrics(
      std::span{*pathResult}, std::span{*dijkstraPathResult}, map, scenario.footprint);
  if (!metrics) {
    fmt::print(stderr, "Metrics error: {}\n", metrics.error().message);
    return 1;
  }

  const auto artifactStatus =
      ad::planning_test::saveArtifacts(scenario, std::span{*pathResult}, *metrics, map);
  if (!artifactStatus) {
    fmt::print(stderr, "Artifact save error: {}\n", artifactStatus.error().message);
    return 1;
  }

  fmt::print("Planning test completed. PNG: test/planning/logs/planning_result.png, metrics: "
             "test/planning/logs/metrics.json\n");
  return 0;
}
