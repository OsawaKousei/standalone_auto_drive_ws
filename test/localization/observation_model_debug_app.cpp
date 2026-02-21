#include "features/localization/localizer_factory.hpp"
#include "features/localization/localizer_util.hpp"
#include "features/localization/observation_model/hough_line_extractor.hpp"
#include "features/localization/observation_model/hough_observation_model.hpp"
#include "features/localization/observation_model/ransac_core.hpp"
#include "features/simulation/simulation_factory.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/text_config.hpp"
#include "shared/types.hpp"

#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <fstream>
#include <iomanip>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ad::observation_debug {

constexpr auto kLogPrecision = 8;
constexpr auto kSeedMul = std::uint32_t{0x9e3779b9U};

struct AlgorithmSpec {
  std::string algorithm;
  std::string configPath;
};

struct TestScenarioConfig {
  std::string baseDir;
  std::string mapYamlPath;
  types::Pose start;
  AlgorithmSpec localization;
  AlgorithmSpec lidarSensor;
};

struct ProgramOptions {
  std::string scenarioPath;
  std::string outPath;
};

struct LineDiagnostic {
  std::size_t lineIndex;
  int bucketCount;
  bool houghValid;
  int houghInliers;
  double houghMse;
  bool houghGatePass;
  double houghResidualRho;
  double houghResidualAlpha;
  std::optional<types::LineSegment> houghSegmentWorld;
  bool ransacValid;
  int ransacInliers;
  double ransacMse;
  bool ransacGatePass;
  double ransacResidualRho;
  double ransacResidualAlpha;
  std::optional<types::LineSegment> ransacSegmentWorld;
};

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
  return (std::filesystem::path{std::string{baseDir}} / candidate).lexically_normal().string();
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

  const auto mapYamlPath = requiredString(*cfg, "map", "yaml_path");
  if (!mapYamlPath) {
    return tl::make_unexpected(mapYamlPath.error());
  }

  const auto start = parsePose(*cfg, "robot.start");
  if (!start) {
    return tl::make_unexpected(start.error());
  }

  const auto localization = parseAlgorithmSpec(*cfg, "localization");
  if (!localization) {
    return tl::make_unexpected(localization.error());
  }

  const auto lidarSensor = parseAlgorithmSpec(*cfg, "lidar_sensor");
  if (!lidarSensor) {
    return tl::make_unexpected(lidarSensor.error());
  }

  return TestScenarioConfig{.baseDir = baseDirNormalized,
                            .mapYamlPath = *mapYamlPath,
                            .start = *start,
                            .localization = *localization,
                            .lidarSensor = *lidarSensor};
}

[[nodiscard]] auto parseProgramOptions(std::span<char *> arguments) -> Result<ProgramOptions> {
  auto scenarioPath = std::string{"test/localization/configs/localization.toml"};
  auto outPath = std::string{"test/localization/logs/observation_model_debug.csv"};

  for (std::size_t index = 1; index < arguments.size(); ++index) {
    const auto argument = std::string_view{arguments[index]};
    if (argument == "-h" || argument == "--help") {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput,
                .message = "Usage: observation_model_debug_app [scenario.toml] [--out path]"});
    }
    if (argument == "--out") {
      if (index + 1 >= arguments.size()) {
        return tl::make_unexpected(
            Error{.code = ErrorCode::InvalidInput, .message = "--out requires a path value."});
      }
      outPath = std::string{arguments[++index]};
      continue;
    }
    if (!argument.empty() && argument.front() == '-') {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Unknown option: " + std::string{argument}});
    }
    scenarioPath = std::string{argument};
  }

  return ProgramOptions{.scenarioPath = scenarioPath, .outPath = outPath};
}

[[nodiscard]] auto parseHoughConfig(const config::TextConfig &cfg)
    -> Result<localization::HoughConfig> {
  const auto thetaBins = requiredInt(cfg, "hough", "theta_bins");
  if (!thetaBins) {
    return tl::make_unexpected(thetaBins.error());
  }
  const auto rhoBins = requiredInt(cfg, "hough", "rho_bins");
  if (!rhoBins) {
    return tl::make_unexpected(rhoBins.error());
  }
  const auto minVotes = requiredInt(cfg, "hough", "min_votes");
  if (!minVotes) {
    return tl::make_unexpected(minVotes.error());
  }
  const auto maxLines = requiredInt(cfg, "hough", "max_lines");
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto inlierDistance = requiredDouble(cfg, "hough", "inlier_distance");
  if (!inlierDistance) {
    return tl::make_unexpected(inlierDistance.error());
  }
  const auto minSegmentLength = requiredDouble(cfg, "hough", "min_segment_length");
  if (!minSegmentLength) {
    return tl::make_unexpected(minSegmentLength.error());
  }
  const auto mergeRho = requiredDouble(cfg, "hough", "merge_rho");
  if (!mergeRho) {
    return tl::make_unexpected(mergeRho.error());
  }
  const auto mergeTheta = requiredDouble(cfg, "hough", "merge_theta");
  if (!mergeTheta) {
    return tl::make_unexpected(mergeTheta.error());
  }

  return localization::HoughConfig{.thetaBins = *thetaBins,
                                   .rhoBins = *rhoBins,
                                   .minVotes = *minVotes,
                                   .maxLines = *maxLines,
                                   .inlierDistance = *inlierDistance,
                                   .minSegmentLength = *minSegmentLength,
                                   .mergeRho = *mergeRho,
                                   .mergeTheta = *mergeTheta};
}

[[nodiscard]] auto parseObservationModelConfig(const config::TextConfig &cfg)
    -> Result<localization::HoughObservationModelConfig> {
  const auto houghCfg = parseHoughConfig(cfg);
  if (!houghCfg) {
    return tl::make_unexpected(houghCfg.error());
  }

  const auto measurementNoiseRange = requiredDouble(cfg, "ekf", "measurement_noise_range");
  if (!measurementNoiseRange) {
    return tl::make_unexpected(measurementNoiseRange.error());
  }
  const auto measurementNoiseAngle = requiredDouble(cfg, "ekf", "measurement_noise_angle");
  if (!measurementNoiseAngle) {
    return tl::make_unexpected(measurementNoiseAngle.error());
  }
  const auto maxAssociationDistance =
      requiredDouble(cfg, "association", "max_association_distance");
  if (!maxAssociationDistance) {
    return tl::make_unexpected(maxAssociationDistance.error());
  }
  const auto segmentMargin = requiredDouble(cfg, "association", "segment_margin");
  if (!segmentMargin) {
    return tl::make_unexpected(segmentMargin.error());
  }
  const auto gateThreshold = requiredDouble(cfg, "association", "gate_threshold");
  if (!gateThreshold) {
    return tl::make_unexpected(gateThreshold.error());
  }
  const auto minObservations = requiredInt(cfg, "association", "min_observations");
  if (!minObservations) {
    return tl::make_unexpected(minObservations.error());
  }

  return localization::HoughObservationModelConfig{
      .hough = *houghCfg,
      .measurementNoiseRange = *measurementNoiseRange,
      .measurementNoiseAngle = *measurementNoiseAngle,
      .maxAssociationDistance = *maxAssociationDistance,
      .segmentMargin = *segmentMargin,
      .gateThreshold = *gateThreshold,
      .minObservations = static_cast<std::size_t>(*minObservations)};
}

[[nodiscard]] auto parseRansacConfig(const config::TextConfig &cfg)
    -> Result<localization::RansacConfig> {
  const auto maxIterations = requiredInt(cfg, "ransac", "max_iterations");
  if (!maxIterations) {
    return tl::make_unexpected(maxIterations.error());
  }
  const auto inlierDistance = requiredDouble(cfg, "ransac", "inlier_distance");
  if (!inlierDistance) {
    return tl::make_unexpected(inlierDistance.error());
  }
  const auto minInliers = requiredInt(cfg, "ransac", "min_inliers");
  if (!minInliers) {
    return tl::make_unexpected(minInliers.error());
  }
  const auto minInlierRatio = requiredDouble(cfg, "ransac", "min_inlier_ratio");
  if (!minInlierRatio) {
    return tl::make_unexpected(minInlierRatio.error());
  }

  return localization::RansacConfig{.maxIterations = *maxIterations,
                                    .inlierDistance = *inlierDistance,
                                    .minInliers = static_cast<std::size_t>(*minInliers),
                                    .minInlierRatio = *minInlierRatio};
}

[[nodiscard]] auto toWorldPoint(const types::Pose &pose, const types::Point &point)
    -> types::Point {
  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);
  return types::Point{.x = pose.x + (cosTheta * point.x) - (sinTheta * point.y),
                      .y = pose.y + (sinTheta * point.x) + (cosTheta * point.y)};
}

[[nodiscard]] auto scanToWorldPoints(const types::Pose &pose, const types::LidarScan &scan)
    -> std::vector<types::Point> {
  auto points = std::vector<types::Point>{};
  points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }
    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    const auto local = types::Point{.x = range * std::cos(angle), .y = range * std::sin(angle)};
    points.push_back(toWorldPoint(pose, local));
  }
  return points;
}

[[nodiscard]] auto serializePoint(const types::Point &point) -> std::string {
  return fmt::format("{:.8f}:{:.8f}", point.x, point.y);
}

[[nodiscard]] auto serializeSegment(const std::optional<types::LineSegment> &segment)
    -> std::string {
  if (!segment.has_value()) {
    return "";
  }
  return serializePoint(segment->start) + ";" + serializePoint(segment->end);
}

[[nodiscard]] auto serializePoints(std::span<const types::Point> points) -> std::string {
  auto output = std::string{};
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (index != 0U) {
      output.push_back(';');
    }
    output += serializePoint(points[index]);
  }
  return output;
}

[[nodiscard]] auto
buildBuckets(const types::LidarScan &scan, const std::vector<localization::util::MapLine> &mapLines,
             const types::Pose &pose, const localization::HoughObservationModelConfig &config)
    -> std::vector<std::vector<types::Point>> {
  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);
  auto buckets = std::vector<std::vector<types::Point>>(mapLines.size());

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }

    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    const auto localX = range * std::cos(angle);
    const auto localY = range * std::sin(angle);

    const auto mapX = pose.x + (cosTheta * localX) - (sinTheta * localY);
    const auto mapY = pose.y + (sinTheta * localX) + (cosTheta * localY);

    std::size_t bestIndex = mapLines.size();
    auto bestDistance = std::optional<double>{};
    for (std::size_t lineIndex = 0; lineIndex < mapLines.size(); ++lineIndex) {
      const auto &line = mapLines[lineIndex];
      const auto projection = (line.directionX * mapX) + (line.directionY * mapY);
      if (projection < (line.minProjection - config.segmentMargin) ||
          projection > (line.maxProjection + config.segmentMargin)) {
        continue;
      }

      const auto lineNormalX = std::cos(line.model.alpha);
      const auto lineNormalY = std::sin(line.model.alpha);
      const auto distance = std::abs((lineNormalX * mapX) + (lineNormalY * mapY) - line.model.rho);
      if (distance > config.maxAssociationDistance) {
        continue;
      }
      if (!bestDistance || distance < *bestDistance) {
        bestDistance = distance;
        bestIndex = lineIndex;
      }
    }

    if (bestIndex >= mapLines.size()) {
      continue;
    }

    buckets[bestIndex].push_back(types::Point{.x = localX, .y = localY});
  }

  return buckets;
}

[[nodiscard]] auto buildSegmentFromFitLocal(const std::vector<types::Point> &pointsLocal,
                                            const localization::util::LineFit &fit,
                                            const types::Pose &pose, double inlierDistance)
    -> std::optional<types::LineSegment> {
  if (pointsLocal.empty()) {
    return std::nullopt;
  }

  const auto normalX = std::cos(fit.model.alpha);
  const auto normalY = std::sin(fit.model.alpha);
  const auto tangentX = -normalY;
  const auto tangentY = normalX;

  auto minProjection = std::optional<double>{};
  auto maxProjection = std::optional<double>{};
  for (const auto &point : pointsLocal) {
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - fit.model.rho);
    if (distance > inlierDistance) {
      continue;
    }
    const auto projection = (tangentX * point.x) + (tangentY * point.y);
    if (!minProjection || projection < *minProjection) {
      minProjection = projection;
    }
    if (!maxProjection || projection > *maxProjection) {
      maxProjection = projection;
    }
  }

  if (!minProjection || !maxProjection) {
    return std::nullopt;
  }

  const auto startLocal =
      types::Point{.x = (tangentX * *minProjection) + (normalX * fit.model.rho),
                   .y = (tangentY * *minProjection) + (normalY * fit.model.rho)};
  const auto endLocal = types::Point{.x = (tangentX * *maxProjection) + (normalX * fit.model.rho),
                                     .y = (tangentY * *maxProjection) + (normalY * fit.model.rho)};
  return types::LineSegment{.start = toWorldPoint(pose, startLocal),
                            .end = toWorldPoint(pose, endLocal)};
}

[[nodiscard]] auto makeObservation(const localization::util::MapLine &mapLine,
                                   const localization::util::LineFit &fit, const types::Pose &pose,
                                   const localization::HoughObservationModelConfig &config,
                                   std::size_t supportCount)
    -> localization::util::LineObservation {
  constexpr double kReferencePoints = 40.0;
  constexpr double kMinRangeVarianceFactor = 0.25;
  constexpr double kMinAngleVarianceFactor = 0.25;
  constexpr double kAngleMseScale = 0.1;

  auto observation = localization::util::makeExpectedLine(mapLine.model, pose);
  observation.observed = fit.model;

  const auto pointCount = std::max(1.0, static_cast<double>(supportCount));
  const auto baseRangeVar = config.measurementNoiseRange * config.measurementNoiseRange;
  const auto baseAngleVar = config.measurementNoiseAngle * config.measurementNoiseAngle;
  const auto scale = std::max(1.0, kReferencePoints / pointCount);
  const auto minRangeVar = baseRangeVar * kMinRangeVarianceFactor;
  const auto minAngleVar = baseAngleVar * kMinAngleVarianceFactor;
  observation.rangeVariance = std::max(minRangeVar, (baseRangeVar * scale) + fit.mse);
  observation.angleVariance =
      std::max(minAngleVar, (baseAngleVar * scale) + (fit.mse * kAngleMseScale));
  return observation;
}

[[nodiscard]] auto evaluateLine(std::size_t lineIndex, const localization::util::MapLine &mapLine,
                                const std::vector<types::Point> &bucket, const types::Pose &pose,
                                const localization::HoughObservationModelConfig &houghConfig,
                                const localization::RansacConfig &ransacConfig,
                                const localization::CovarianceMatrix &covariance)
    -> LineDiagnostic {
  auto diagnostic = LineDiagnostic{.lineIndex = lineIndex,
                                   .bucketCount = static_cast<int>(bucket.size()),
                                   .houghValid = false,
                                   .houghInliers = 0,
                                   .houghMse = 0.0,
                                   .houghGatePass = false,
                                   .houghResidualRho = 0.0,
                                   .houghResidualAlpha = 0.0,
                                   .houghSegmentWorld = std::nullopt,
                                   .ransacValid = false,
                                   .ransacInliers = 0,
                                   .ransacMse = 0.0,
                                   .ransacGatePass = false,
                                   .ransacResidualRho = 0.0,
                                   .ransacResidualAlpha = 0.0,
                                   .ransacSegmentWorld = std::nullopt};

  const auto houghFit = localization::util::fitLine(bucket);
  if (houghFit) {
    diagnostic.houghValid = true;
    diagnostic.houghInliers = static_cast<int>(houghFit->pointCount);
    diagnostic.houghMse = houghFit->mse;
    const auto observation =
        makeObservation(mapLine, *houghFit, pose, houghConfig, houghFit->pointCount);
    diagnostic.houghGatePass = localization::util::gateLineObservation(
        observation, localization::util::ObservationGateConfig{
                         .covariance = covariance, .threshold = houghConfig.gateThreshold});
    diagnostic.houghResidualRho = observation.observed.rho - observation.expected.rho;
    diagnostic.houghResidualAlpha =
        localization::util::normalizeAngle(observation.observed.alpha - observation.expected.alpha);
    if (const auto seg =
            buildSegmentFromFitLocal(bucket, *houghFit, pose, houghConfig.hough.inlierDistance);
        seg.has_value()) {
      diagnostic.houghSegmentWorld.emplace(*seg);
    }
  }

  const auto seed = static_cast<std::uint32_t>((lineIndex + 1U) * kSeedMul) ^
                    static_cast<std::uint32_t>(bucket.size());
  const auto ransacFit = localization::ransac::fitLineToPoints(bucket, ransacConfig, seed);
  if (ransacFit) {
    diagnostic.ransacValid = true;
    diagnostic.ransacInliers = static_cast<int>(ransacFit->inlierCount);
    diagnostic.ransacMse = ransacFit->fit.mse;
    const auto observation =
        makeObservation(mapLine, ransacFit->fit, pose, houghConfig, ransacFit->inlierCount);
    diagnostic.ransacGatePass = localization::util::gateLineObservation(
        observation, localization::util::ObservationGateConfig{
                         .covariance = covariance, .threshold = houghConfig.gateThreshold});
    diagnostic.ransacResidualRho = observation.observed.rho - observation.expected.rho;
    diagnostic.ransacResidualAlpha =
        localization::util::normalizeAngle(observation.observed.alpha - observation.expected.alpha);
    if (const auto seg =
            buildSegmentFromFitLocal(bucket, ransacFit->fit, pose, ransacConfig.inlierDistance);
        seg.has_value()) {
      diagnostic.ransacSegmentWorld.emplace(*seg);
    }
  }

  return diagnostic;
}

} // namespace ad::observation_debug

auto main(int argc, char **argv) -> int {
  const auto arguments = std::span<char *>{argv, static_cast<std::size_t>(argc)};
  const auto optionsResult = ad::observation_debug::parseProgramOptions(arguments);
  if (!optionsResult) {
    fmt::print(stderr, "Argument error: {}\n", optionsResult.error().message);
    return 1;
  }

  const auto scenarioResult = ad::observation_debug::loadScenario(optionsResult->scenarioPath);
  if (!scenarioResult) {
    fmt::print(stderr, "Scenario error: {}\n", scenarioResult.error().message);
    return 1;
  }
  const auto &scenario = *scenarioResult;

  const auto mapPath = ad::observation_debug::resolvePath(scenario.baseDir, scenario.mapYamlPath);
  const auto mapResult = ad::loadMapFromYaml(mapPath);
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }
  const auto &map = *mapResult;

  const auto localizationCfgResult = ad::config::loadTextConfig(
      ad::observation_debug::resolvePath(scenario.baseDir, scenario.localization.configPath));
  if (!localizationCfgResult) {
    fmt::print(stderr, "Localization config load error: {}\n",
               localizationCfgResult.error().message);
    return 1;
  }

  const auto lidarCfgResult = ad::config::loadTextConfig(
      ad::observation_debug::resolvePath(scenario.baseDir, scenario.lidarSensor.configPath));
  if (!lidarCfgResult) {
    fmt::print(stderr, "Lidar config load error: {}\n", lidarCfgResult.error().message);
    return 1;
  }

  const auto houghObsConfig =
      ad::observation_debug::parseObservationModelConfig(*localizationCfgResult);
  if (!houghObsConfig) {
    fmt::print(stderr, "Hough observation config parse error: {}\n",
               houghObsConfig.error().message);
    return 1;
  }
  const auto ransacCfg = ad::observation_debug::parseRansacConfig(*localizationCfgResult);
  if (!ransacCfg) {
    fmt::print(stderr, "RANSAC config parse error: {}\n", ransacCfg.error().message);
    return 1;
  }

  const auto covariance = ad::localization::parseInitialCovarianceFromConfig(
      std::optional<ad::config::TextConfig>{*localizationCfgResult});
  if (!covariance) {
    fmt::print(stderr, "Initial covariance error: {}\n", covariance.error().message);
    return 1;
  }

  auto lidarSensorResult = ad::simulation::createLidarSensorFromConfig(
      scenario.lidarSensor.algorithm, std::optional<ad::config::TextConfig>{*lidarCfgResult});
  if (!lidarSensorResult) {
    fmt::print(stderr, "Lidar sensor create error: {}\n", lidarSensorResult.error().message);
    return 1;
  }

  const auto mapLines = ad::localization::hough::extractMapLinesFromMap(map, houghObsConfig->hough);
  if (!mapLines) {
    fmt::print(stderr, "Map line extraction error: {}\n", mapLines.error().message);
    return 1;
  }

  const auto scanResult = (*lidarSensorResult)->simulate(map, scenario.start);
  if (!scanResult) {
    fmt::print(stderr, "Scan simulation error: {}\n", scanResult.error().message);
    return 1;
  }

  const auto scanWorldPoints =
      ad::observation_debug::scanToWorldPoints(scenario.start, *scanResult);
  const auto buckets =
      ad::observation_debug::buildBuckets(*scanResult, *mapLines, scenario.start, *houghObsConfig);

  auto diagnostics = std::vector<ad::observation_debug::LineDiagnostic>{};
  diagnostics.reserve(mapLines->size());
  for (std::size_t lineIndex = 0; lineIndex < mapLines->size(); ++lineIndex) {
    diagnostics.push_back(ad::observation_debug::evaluateLine(
        lineIndex, (*mapLines)[lineIndex], buckets[lineIndex], scenario.start, *houghObsConfig,
        *ransacCfg, *covariance));
  }

  std::error_code fsError;
  std::filesystem::create_directories(std::filesystem::path{optionsResult->outPath}.parent_path(),
                                      fsError);
  if (fsError) {
    fmt::print(stderr, "Output directory error: {}\n", fsError.message());
    return 1;
  }

  auto out = std::ofstream{optionsResult->outPath};
  if (!out.is_open()) {
    fmt::print(stderr, "Output file error: failed to open {}\n", optionsResult->outPath);
    return 1;
  }

  out << std::fixed << std::setprecision(ad::observation_debug::kLogPrecision);
  out << "# observation_model_debug\n";
  out << "# scenario_config=" << optionsResult->scenarioPath << "\n";
  out << "# map_line_count=" << mapLines->size() << "\n";
  out << "# scan_point_count=" << scanWorldPoints.size() << "\n";
  out << "# scan_world=" << ad::observation_debug::serializePoints(scanWorldPoints) << "\n";
  out << "# columns: line_idx,map_seg,bucket_count,hough_valid,hough_inliers,hough_mse,"
         "hough_gate,hough_res_rho,hough_res_alpha,hough_seg,ransac_valid,ransac_inliers,"
         "ransac_mse,ransac_gate,ransac_res_rho,ransac_res_alpha,ransac_seg\n";
  out << "line_idx,map_seg,bucket_count,hough_valid,hough_inliers,hough_mse,hough_gate,"
         "hough_res_rho,hough_res_alpha,hough_seg,ransac_valid,ransac_inliers,ransac_mse,"
         "ransac_gate,ransac_res_rho,ransac_res_alpha,ransac_seg\n";

  for (std::size_t lineIndex = 0; lineIndex < diagnostics.size(); ++lineIndex) {
    const auto &diag = diagnostics[lineIndex];
    const auto &mapLine = (*mapLines)[lineIndex];
    const auto mapSegment = ad::observation_debug::serializePoint(mapLine.segment.start) + ";" +
                            ad::observation_debug::serializePoint(mapLine.segment.end);

    out << lineIndex << ',' << mapSegment << ',' << diag.bucketCount << ','
        << (diag.houghValid ? 1 : 0) << ',' << diag.houghInliers << ',' << diag.houghMse << ','
        << (diag.houghGatePass ? 1 : 0) << ',' << diag.houghResidualRho << ','
        << diag.houghResidualAlpha << ','
        << ad::observation_debug::serializeSegment(diag.houghSegmentWorld) << ','
        << (diag.ransacValid ? 1 : 0) << ',' << diag.ransacInliers << ',' << diag.ransacMse << ','
        << (diag.ransacGatePass ? 1 : 0) << ',' << diag.ransacResidualRho << ','
        << diag.ransacResidualAlpha << ','
        << ad::observation_debug::serializeSegment(diag.ransacSegmentWorld) << '\n';
  }

  fmt::print("Saved observation model debug CSV: {}\n", optionsResult->outPath);
  return 0;
}
