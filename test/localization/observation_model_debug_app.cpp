#include "features/localization/i_observation_model.hpp"
#include "features/localization/localization_config.hpp"
#include "features/localization/observation_model/ransac_config.hpp"
#include "features/localization/observation_model/ransac_line_association_model.hpp"
#include "features/localization/observation_model/simple_line_association_model.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/text_config.hpp"
#include "shared/types.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fmt/core.h>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ad::observation_model_debug {

struct ProgramOptions {
  std::string logPath = "test/localization/logs/localization_test.log";
  std::string scenarioPath = "test/localization/configs/localization.toml";
  std::string configAPath = "test/localization/configs/localization/ekf_hough.toml";
  std::string configBPath = "test/localization/configs/localization/ekf_hough_ransac.toml";
  std::string labelA = "hough";
  std::string labelB = "hough_ransac";
  std::string outputCsvPath = "test/localization/logs/observation_model_eval.csv";
  std::string outputJsonPath = "test/localization/logs/observation_model_eval_metrics.json";
};

struct InputFrame {
  types::LidarScan scan;
  types::Pose predictedPose;
  localization::CovarianceMatrix predictedCovariance;
};

struct EvaluationRecord {
  std::size_t frameIndex = 0U;
  std::string modelLabel;
  std::string status;
  double score = 0.0;
  double residualRmse = 0.0;
  double nis = 0.0;
  int measurementCount = 0;
  double runtimeMs = 0.0;
  std::string error;
};

struct EvalSummary {
  std::string label;
  std::size_t totalFrames = 0U;
  std::size_t successCount = 0U;
  std::size_t noUpdateCount = 0U;
  std::size_t errorCount = 0U;
  double scoreSum = 0.0;
  double residualRmseSum = 0.0;
  double nisSum = 0.0;
  std::size_t nisCount = 0U;
  std::size_t measurementCountSum = 0U;
  double runtimeMsSum = 0.0;
  std::vector<double> runtimeSamplesMs{};
};

[[nodiscard]] auto resolvePath(std::string_view baseDir, std::string_view path) -> std::string {
  const auto candidate = std::filesystem::path{std::string{path}};
  if (candidate.is_absolute()) {
    return candidate.lexically_normal().string();
  }
  const auto resolved = std::filesystem::path{std::string{baseDir}} / candidate;
  return resolved.lexically_normal().string();
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

[[nodiscard]] auto optionalInt(const config::TextConfig &cfg, std::string_view section,
                               std::string_view key, int defaultValue) -> Result<int> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return defaultValue;
  }
  return config::parseIntValue(*raw);
}

[[nodiscard]] auto optionalDouble(const config::TextConfig &cfg, std::string_view section,
                                  std::string_view key, double defaultValue) -> Result<double> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return defaultValue;
  }
  return config::parseDoubleValue(*raw);
}

[[nodiscard]] auto parseSimpleLineAssociationConfig(const config::TextConfig &cfg)
    -> Result<localization::SimpleLineAssociationModelConfig> {
  const auto maxLines = [&]() -> Result<int> {
    if (const auto raw = cfg.findRaw("line_extraction", "max_lines")) {
      return config::parseIntValue(*raw);
    }
    return requiredInt(cfg, "hough", "max_lines");
  }();
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto minSegmentLength = [&]() -> Result<double> {
    if (const auto raw = cfg.findRaw("line_extraction", "min_segment_length")) {
      return config::parseDoubleValue(*raw);
    }
    return requiredDouble(cfg, "hough", "min_segment_length");
  }();
  if (!minSegmentLength) {
    return tl::make_unexpected(minSegmentLength.error());
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

  return localization::SimpleLineAssociationModelConfig{
      .mapLineExtraction =
          localization::MapLineExtractionConfig{.maxLines = *maxLines,
                                                .minSegmentLength = *minSegmentLength},
      .measurementNoiseRange = *measurementNoiseRange,
      .measurementNoiseAngle = *measurementNoiseAngle,
      .maxAssociationDistance = *maxAssociationDistance,
      .segmentMargin = *segmentMargin,
      .gateThreshold = *gateThreshold,
      .minObservations = static_cast<std::size_t>(*minObservations)};
}

[[nodiscard]] auto createObservationModel(const types::MapData &map, std::string_view configPath)
    -> Result<std::unique_ptr<localization::IObservationModel>> {
  const auto cfg = config::loadTextConfig(configPath);
  if (!cfg) {
    return tl::make_unexpected(cfg.error());
  }

  auto observationModelType = std::string{"simple_line_association"};
  if (const auto raw = cfg->findRaw("", "observation_model"); raw) {
    const auto parsed = config::parseQuotedString(*raw);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    observationModelType = *parsed;
  }

  const auto simpleConfig = parseSimpleLineAssociationConfig(*cfg);
  if (!simpleConfig) {
    return tl::make_unexpected(simpleConfig.error());
  }

  if (observationModelType == "hough_line" || observationModelType == "simple_line_association") {
    auto model = localization::SimpleLineAssociationModel::create(map, *simpleConfig);
    if (!model) {
      return tl::make_unexpected(model.error());
    }
    std::unique_ptr<localization::IObservationModel> base = std::move(*model);
    return base;
  }

  if (observationModelType == "hough_ransac_line" ||
      observationModelType == "ransac_line_association") {
    const auto inlierDistance = [&]() -> Result<double> {
      if (const auto raw = cfg->findRaw("line_extraction", "inlier_distance")) {
        return config::parseDoubleValue(*raw);
      }
      return requiredDouble(*cfg, "hough", "inlier_distance");
    }();
    if (!inlierDistance) {
      return tl::make_unexpected(inlierDistance.error());
    }
    const auto mergeRho = [&]() -> Result<double> {
      if (const auto raw = cfg->findRaw("line_extraction", "merge_rho")) {
        return config::parseDoubleValue(*raw);
      }
      return requiredDouble(*cfg, "hough", "merge_rho");
    }();
    if (!mergeRho) {
      return tl::make_unexpected(mergeRho.error());
    }
    const auto mergeTheta = [&]() -> Result<double> {
      if (const auto raw = cfg->findRaw("line_extraction", "merge_theta")) {
        return config::parseDoubleValue(*raw);
      }
      return requiredDouble(*cfg, "hough", "merge_theta");
    }();
    if (!mergeTheta) {
      return tl::make_unexpected(mergeTheta.error());
    }
    const auto maxIterations = optionalInt(*cfg, "ransac", "max_iterations", 80);
    if (!maxIterations) {
      return tl::make_unexpected(maxIterations.error());
    }
    const auto minInliers = optionalInt(*cfg, "ransac", "min_inliers", 8);
    if (!minInliers) {
      return tl::make_unexpected(minInliers.error());
    }
    const auto minInlierRatio = optionalDouble(*cfg, "ransac", "min_inlier_ratio", 0.35);
    if (!minInlierRatio) {
      return tl::make_unexpected(minInlierRatio.error());
    }

    const auto configValue = localization::RansacLineAssociationModelConfig{
        .baseObservation = *simpleConfig,
        .ransacLineExtraction =
            localization::RansacLineExtractionConfig{
                .maxLines = simpleConfig->mapLineExtraction.maxLines,
                .inlierDistance = *inlierDistance,
                .minSegmentLength = simpleConfig->mapLineExtraction.minSegmentLength,
                .mergeRho = *mergeRho,
                .mergeTheta = *mergeTheta},
        .ransac = localization::RansacConfig{.maxIterations = *maxIterations,
                                             .minInliers = static_cast<std::size_t>(*minInliers),
                                             .minInlierRatio = *minInlierRatio}};
    auto model = localization::RansacLineAssociationModel::create(map, configValue);
    if (!model) {
      return tl::make_unexpected(model.error());
    }
    std::unique_ptr<localization::IObservationModel> base = std::move(*model);
    return base;
  }

  return tl::make_unexpected(
      Error{.code = ErrorCode::InvalidInput,
            .message = "Unsupported observation model: " + observationModelType});
}

[[nodiscard]] auto split(std::string_view text, char delimiter) -> std::vector<std::string> {
  auto out = std::vector<std::string>{};
  std::size_t start = 0U;
  while (start <= text.size()) {
    const auto end = text.find(delimiter, start);
    if (end == std::string_view::npos) {
      out.emplace_back(text.substr(start));
      break;
    }
    out.emplace_back(text.substr(start, end - start));
    start = end + 1U;
  }
  return out;
}

[[nodiscard]] auto trim(std::string_view text) -> std::string {
  auto begin = std::size_t{0U};
  auto end = text.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
    ++begin;
  }
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1U])) != 0) {
    --end;
  }
  return std::string{text.substr(begin, end - begin)};
}

[[nodiscard]] auto parseArgs(int argc, char **argv) -> Result<ProgramOptions> {
  auto options = ProgramOptions{};
  for (auto index = 1; index < argc; ++index) {
    const auto arg = std::string_view{argv[index]};
    const auto readValue = [&](std::string_view name) -> Result<std::string> {
      if ((index + 1) >= argc) {
        return tl::make_unexpected(
            Error{.code = ErrorCode::InvalidInput,
                  .message = "Missing value for argument: " + std::string{name}});
      }
      ++index;
      return std::string{argv[index]};
    };

    if (arg == "--log") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.logPath = *value;
    } else if (arg == "--scenario") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.scenarioPath = *value;
    } else if (arg == "--config-a") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.configAPath = *value;
    } else if (arg == "--config-b") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.configBPath = *value;
    } else if (arg == "--label-a") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.labelA = *value;
    } else if (arg == "--label-b") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.labelB = *value;
    } else if (arg == "--out-csv") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.outputCsvPath = *value;
    } else if (arg == "--out-json") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.outputJsonPath = *value;
    } else {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput,
                .message = "Unknown argument: " + std::string{arg} +
                           "\nUsage: observation_model_debug_app "
                           "[--log path] [--scenario path] [--config-a path] [--config-b path] "
                           "[--label-a name] [--label-b name] [--out-csv path] [--out-json path]"});
    }
  }

  return options;
}

[[nodiscard]] auto parseRanges(std::string_view raw) -> Result<std::vector<double>> {
  auto ranges = std::vector<double>{};
  if (raw.empty()) {
    return ranges;
  }

  for (const auto &token : split(raw, ';')) {
    if (token.empty()) {
      continue;
    }
    try {
      ranges.push_back(std::stod(token));
    } catch (const std::exception &) {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Failed to parse scan range token: " + token});
    }
  }
  return ranges;
}

[[nodiscard]] auto parseDoubleCell(const std::vector<std::string> &row, std::size_t index,
                                   std::string_view column) -> Result<double> {
  if (index >= row.size()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "CSV index out of range for column: " + std::string{column}});
  }
  try {
    return std::stod(row[index]);
  } catch (const std::exception &) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Failed to parse double column: " + std::string{column}});
  }
}

[[nodiscard]] auto parseIntCell(const std::vector<std::string> &row, std::size_t index,
                                std::string_view column) -> Result<int> {
  if (index >= row.size()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "CSV index out of range for column: " + std::string{column}});
  }
  try {
    return std::stoi(row[index]);
  } catch (const std::exception &) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Failed to parse int column: " + std::string{column}});
  }
}

[[nodiscard]] auto findColumn(const std::unordered_map<std::string, std::size_t> &columns,
                              std::string_view name) -> Result<std::size_t> {
  const auto it = columns.find(std::string{name});
  if (it == columns.end()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Required CSV column is missing: " + std::string{name}});
  }
  return it->second;
}

[[nodiscard]] auto loadFramesFromLog(std::string_view logPath) -> Result<std::vector<InputFrame>> {
  auto stream = std::ifstream{std::string{logPath}};
  if (!stream.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Failed to open log file: " + std::string{logPath}});
  }

  auto columns = std::unordered_map<std::string, std::size_t>{};
  auto columnCount = std::size_t{0U};
  auto headerReady = false;
  auto frames = std::vector<InputFrame>{};
  auto line = std::string{};

  while (std::getline(stream, line)) {
    const auto stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
      continue;
    }

    if (!headerReady) {
      const auto header = split(stripped, ',');
      for (std::size_t index = 0; index < header.size(); ++index) {
        columns.emplace(trim(header[index]), index);
      }
      columnCount = header.size();
      headerReady = true;
      continue;
    }

    const auto row = split(stripped, ',');
    if (row.size() != columnCount) {
      continue;
    }

    const auto lidarUpdatedIdx = findColumn(columns, "lidar_updated");
    if (!lidarUpdatedIdx) {
      return tl::make_unexpected(lidarUpdatedIdx.error());
    }
    const auto lidarUpdated = parseIntCell(row, *lidarUpdatedIdx, "lidar_updated");
    if (!lidarUpdated) {
      return tl::make_unexpected(lidarUpdated.error());
    }
    if (*lidarUpdated != 1) {
      continue;
    }

    const auto scanCountIdx = findColumn(columns, "scan_count");
    const auto minAngleIdx = findColumn(columns, "scan_min_angle");
    const auto angleIncIdx = findColumn(columns, "scan_angle_inc");
    const auto maxRangeIdx = findColumn(columns, "scan_max_range");
    const auto scanRangesIdx = findColumn(columns, "scan_ranges");
    const auto estXIdx = findColumn(columns, "est_x");
    const auto estYIdx = findColumn(columns, "est_y");
    const auto estThetaIdx = findColumn(columns, "est_theta");
    const auto covXxIdx = findColumn(columns, "cov_xx");
    const auto covYyIdx = findColumn(columns, "cov_yy");
    const auto covTtIdx = findColumn(columns, "cov_tt");
    if (!scanCountIdx || !minAngleIdx || !angleIncIdx || !maxRangeIdx || !scanRangesIdx ||
        !estXIdx || !estYIdx || !estThetaIdx || !covXxIdx || !covYyIdx || !covTtIdx) {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Failed to resolve required CSV columns."});
    }

    const auto scanCount = parseIntCell(row, *scanCountIdx, "scan_count");
    const auto minAngle = parseDoubleCell(row, *minAngleIdx, "scan_min_angle");
    const auto angleInc = parseDoubleCell(row, *angleIncIdx, "scan_angle_inc");
    const auto maxRange = parseDoubleCell(row, *maxRangeIdx, "scan_max_range");
    const auto estX = parseDoubleCell(row, *estXIdx, "est_x");
    const auto estY = parseDoubleCell(row, *estYIdx, "est_y");
    const auto estTheta = parseDoubleCell(row, *estThetaIdx, "est_theta");
    const auto covXx = parseDoubleCell(row, *covXxIdx, "cov_xx");
    const auto covYy = parseDoubleCell(row, *covYyIdx, "cov_yy");
    const auto covTt = parseDoubleCell(row, *covTtIdx, "cov_tt");
    if (!scanCount || !minAngle || !angleInc || !maxRange || !estX || !estY || !estTheta ||
        !covXx || !covYy || !covTt) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse CSV numeric values."});
    }

    const auto ranges = parseRanges(row[*scanRangesIdx]);
    if (!ranges) {
      return tl::make_unexpected(ranges.error());
    }
    if (ranges->empty() || static_cast<int>(ranges->size()) != *scanCount) {
      continue;
    }

    auto covariance = localization::CovarianceMatrix{};
    covariance.setZero();
    covariance(0, 0) = *covXx;
    covariance(1, 1) = *covYy;
    covariance(2, 2) = *covTt;

    frames.push_back(
        InputFrame{.scan = types::LidarScan{.ranges = *ranges,
                                            .minAngle = *minAngle,
                                            .angleIncrement = *angleInc,
                                            .maxRange = *maxRange},
                   .predictedPose = types::Pose{.x = *estX, .y = *estY, .theta = *estTheta},
                   .predictedCovariance = covariance});
  }

  if (frames.empty()) {
    return tl::make_unexpected(Error{.code = ErrorCode::EmptyCollection,
                                     .message = "No valid lidar update frames were found in log."});
  }
  return frames;
}

[[nodiscard]] auto computeNis(const localization::ObservationUpdateInput &input,
                              const localization::CovarianceMatrix &predictedCovariance)
    -> std::optional<double> {
  const auto &residual = input.residual;
  const auto &matrixH = input.measurementMatrix;
  const auto &matrixR = input.measurementNoise;
  if (residual.size() == 0 || matrixH.rows() != residual.size() ||
      matrixR.rows() != residual.size() || matrixR.cols() != residual.size()) {
    return std::nullopt;
  }

  const auto matrixS = (matrixH * predictedCovariance * matrixH.transpose()) + matrixR;
  auto ldlt = Eigen::LDLT<Eigen::MatrixXd>{matrixS};
  if (ldlt.info() != Eigen::Success) {
    return std::nullopt;
  }
  const auto solved = ldlt.solve(residual);
  if (ldlt.info() != Eigen::Success) {
    return std::nullopt;
  }
  return residual.dot(solved);
}

auto accumulate(EvalSummary &summary, const EvaluationRecord &record) -> void {
  ++summary.totalFrames;
  summary.runtimeMsSum += record.runtimeMs;
  summary.runtimeSamplesMs.push_back(record.runtimeMs);

  if (record.status == "success") {
    ++summary.successCount;
    summary.scoreSum += record.score;
    summary.residualRmseSum += record.residualRmse;
    summary.measurementCountSum += static_cast<std::size_t>(record.measurementCount);
    if (record.nis > 0.0) {
      summary.nisSum += record.nis;
      ++summary.nisCount;
    }
  } else if (record.status == "no_update") {
    ++summary.noUpdateCount;
  } else {
    ++summary.errorCount;
  }
}

[[nodiscard]] auto percentile95(std::vector<double> values) -> double {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index =
      static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(values.size())) - 1.0);
  return values[std::min(index, values.size() - 1U)];
}

[[nodiscard]] auto sanitizeError(std::string message) -> std::string {
  std::replace(message.begin(), message.end(), ',', ';');
  return message;
}

auto writeCsv(std::string_view path, const std::vector<EvaluationRecord> &records) -> Result<void> {
  std::filesystem::create_directories(std::filesystem::path{std::string{path}}.parent_path());
  auto stream = std::ofstream{std::string{path}};
  if (!stream.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Failed to open output CSV path: " + std::string{path}});
  }

  stream << "frame,model,status,score,residual_rmse,nis,measurement_count,runtime_ms,error\n";
  stream << std::fixed << std::setprecision(8);
  for (const auto &record : records) {
    stream << record.frameIndex << ',' << record.modelLabel << ',' << record.status << ','
           << record.score << ',' << record.residualRmse << ',' << record.nis << ','
           << record.measurementCount << ',' << record.runtimeMs << ','
           << sanitizeError(record.error) << '\n';
  }
  return {};
}

[[nodiscard]] auto successRate(const EvalSummary &summary) -> double {
  if (summary.totalFrames == 0U) {
    return 0.0;
  }
  return static_cast<double>(summary.successCount) / static_cast<double>(summary.totalFrames);
}

auto writeMetricsJson(std::string_view path, const EvalSummary &summaryA,
                      const EvalSummary &summaryB, std::string_view configAPath,
                      std::string_view configBPath, std::size_t frameCount) -> Result<void> {
  std::filesystem::create_directories(std::filesystem::path{std::string{path}}.parent_path());
  auto stream = std::ofstream{std::string{path}};
  if (!stream.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Failed to open output JSON path: " + std::string{path}});
  }

  const auto meanScoreA = summaryA.successCount > 0U
                              ? (summaryA.scoreSum / static_cast<double>(summaryA.successCount))
                              : 0.0;
  const auto meanScoreB = summaryB.successCount > 0U
                              ? (summaryB.scoreSum / static_cast<double>(summaryB.successCount))
                              : 0.0;
  const auto meanResidualA =
      summaryA.successCount > 0U
          ? (summaryA.residualRmseSum / static_cast<double>(summaryA.successCount))
          : 0.0;
  const auto meanResidualB =
      summaryB.successCount > 0U
          ? (summaryB.residualRmseSum / static_cast<double>(summaryB.successCount))
          : 0.0;
  const auto meanNisA =
      summaryA.nisCount > 0U ? (summaryA.nisSum / static_cast<double>(summaryA.nisCount)) : 0.0;
  const auto meanNisB =
      summaryB.nisCount > 0U ? (summaryB.nisSum / static_cast<double>(summaryB.nisCount)) : 0.0;
  const auto meanRuntimeA =
      summaryA.totalFrames > 0U
          ? (summaryA.runtimeMsSum / static_cast<double>(summaryA.totalFrames))
          : 0.0;
  const auto meanRuntimeB =
      summaryB.totalFrames > 0U
          ? (summaryB.runtimeMsSum / static_cast<double>(summaryB.totalFrames))
          : 0.0;
  const auto meanMeasA = summaryA.successCount > 0U
                             ? (static_cast<double>(summaryA.measurementCountSum) /
                                static_cast<double>(summaryA.successCount))
                             : 0.0;
  const auto meanMeasB = summaryB.successCount > 0U
                             ? (static_cast<double>(summaryB.measurementCountSum) /
                                static_cast<double>(summaryB.successCount))
                             : 0.0;

  stream << std::fixed << std::setprecision(8);
  stream << "{\n";
  stream << fmt::format("  \"frame_count\": {},\n", frameCount);
  stream << "  \"models\": [\n";
  const auto writeOne = [&](const EvalSummary &summary, std::string_view configPath, bool isLast) {
    stream << "    {\n";
    stream << fmt::format("      \"label\": \"{}\",\n", summary.label);
    stream << fmt::format("      \"config_path\": \"{}\",\n", configPath);
    stream << fmt::format("      \"total_frames\": {},\n", summary.totalFrames);
    stream << fmt::format("      \"success_count\": {},\n", summary.successCount);
    stream << fmt::format("      \"no_update_count\": {},\n", summary.noUpdateCount);
    stream << fmt::format("      \"error_count\": {},\n", summary.errorCount);
    stream << fmt::format("      \"success_rate\": {:.8f},\n", successRate(summary));
    stream << fmt::format("      \"mean_score\": {:.8f},\n",
                          summary.successCount > 0U
                              ? (summary.scoreSum / static_cast<double>(summary.successCount))
                              : 0.0);
    stream << fmt::format("      \"mean_residual_rmse\": {:.8f},\n",
                          summary.successCount > 0U ? (summary.residualRmseSum /
                                                       static_cast<double>(summary.successCount))
                                                    : 0.0);
    stream << fmt::format(
        "      \"mean_nis\": {:.8f},\n",
        summary.nisCount > 0U ? (summary.nisSum / static_cast<double>(summary.nisCount)) : 0.0);
    stream << fmt::format("      \"mean_measurement_count\": {:.8f},\n",
                          summary.successCount > 0U
                              ? (static_cast<double>(summary.measurementCountSum) /
                                 static_cast<double>(summary.successCount))
                              : 0.0);
    stream << fmt::format("      \"mean_runtime_ms\": {:.8f},\n",
                          summary.totalFrames > 0U
                              ? (summary.runtimeMsSum / static_cast<double>(summary.totalFrames))
                              : 0.0);
    stream << fmt::format("      \"p95_runtime_ms\": {:.8f}\n",
                          percentile95(summary.runtimeSamplesMs));
    stream << (isLast ? "    }\n" : "    },\n");
  };
  writeOne(summaryA, configAPath, false);
  writeOne(summaryB, configBPath, true);
  stream << "  ],\n";
  stream << "  \"comparison\": {\n";
  stream << fmt::format("    \"success_rate_delta\": {:.8f},\n",
                        successRate(summaryB) - successRate(summaryA));
  stream << fmt::format("    \"mean_score_delta\": {:.8f},\n", meanScoreB - meanScoreA);
  stream << fmt::format("    \"mean_residual_rmse_delta\": {:.8f},\n",
                        meanResidualB - meanResidualA);
  stream << fmt::format("    \"mean_nis_delta\": {:.8f},\n", meanNisB - meanNisA);
  stream << fmt::format("    \"mean_measurement_count_delta\": {:.8f},\n", meanMeasB - meanMeasA);
  stream << fmt::format("    \"mean_runtime_ms_delta\": {:.8f}\n", meanRuntimeB - meanRuntimeA);
  stream << "  }\n";
  stream << "}\n";

  return {};
}

[[nodiscard]] auto evaluateOneFrame(std::size_t frameIndex, std::string_view label,
                                    const localization::IObservationModel &model,
                                    const types::MapData &map, const InputFrame &frame)
    -> EvaluationRecord {
  const auto start = std::chrono::steady_clock::now();
  const auto built =
      model.buildUpdateInput(frame.scan, map, frame.predictedPose, frame.predictedCovariance);
  const auto end = std::chrono::steady_clock::now();
  const auto runtimeMs = std::chrono::duration<double, std::milli>(end - start).count();

  auto record = EvaluationRecord{.frameIndex = frameIndex,
                                 .modelLabel = std::string{label},
                                 .status = "error",
                                 .runtimeMs = runtimeMs,
                                 .error = ""};

  if (!built) {
    record.error = built.error().message;
    return record;
  }

  if (!built->has_value()) {
    record.status = "no_update";
    return record;
  }

  const auto &update = **built;
  record.status = "success";
  record.score = update.score;
  record.measurementCount = static_cast<int>(update.residual.size() / 2);
  record.residualRmse =
      update.residual.size() > 0
          ? std::sqrt(update.residual.squaredNorm() / static_cast<double>(update.residual.size()))
          : 0.0;
  if (const auto nis = computeNis(update, frame.predictedCovariance); nis) {
    record.nis = *nis;
  }
  return record;
}

[[nodiscard]] auto resolveMapYamlPath(std::string_view scenarioPath) -> Result<std::string> {
  const auto scenarioFsPath = std::filesystem::path{std::string{scenarioPath}};
  const auto baseDir = scenarioFsPath.parent_path().empty() ? std::filesystem::path{"."}
                                                            : scenarioFsPath.parent_path();
  const auto cfg = config::loadTextConfig(scenarioFsPath.lexically_normal().string());
  if (!cfg) {
    return tl::make_unexpected(cfg.error());
  }

  const auto mapYamlPath = requiredString(*cfg, "map", "yaml_path");
  if (!mapYamlPath) {
    return tl::make_unexpected(mapYamlPath.error());
  }

  return resolvePath(baseDir.lexically_normal().string(), *mapYamlPath);
}

} // namespace ad::observation_model_debug

auto main(int argc, char **argv) -> int {
  using namespace ad;
  using namespace ad::observation_model_debug;

  const auto options = parseArgs(argc, argv);
  if (!options) {
    fmt::print(stderr, "[observation_model_debug_app] argument error: {}\n",
               options.error().message);
    return 1;
  }

  const auto mapYamlPath = resolveMapYamlPath(options->scenarioPath);
  if (!mapYamlPath) {
    fmt::print(stderr, "[observation_model_debug_app] scenario parse error: {}\n",
               mapYamlPath.error().message);
    return 1;
  }

  const auto map = loadMapFromYaml(*mapYamlPath);
  if (!map) {
    fmt::print(stderr, "[observation_model_debug_app] map load error: {}\n", map.error().message);
    return 1;
  }

  const auto modelA = createObservationModel(*map, options->configAPath);
  if (!modelA) {
    fmt::print(stderr, "[observation_model_debug_app] model A create error: {}\n",
               modelA.error().message);
    return 1;
  }
  const auto modelB = createObservationModel(*map, options->configBPath);
  if (!modelB) {
    fmt::print(stderr, "[observation_model_debug_app] model B create error: {}\n",
               modelB.error().message);
    return 1;
  }

  const auto frames = loadFramesFromLog(options->logPath);
  if (!frames) {
    fmt::print(stderr, "[observation_model_debug_app] log parse error: {}\n",
               frames.error().message);
    return 1;
  }

  auto records = std::vector<EvaluationRecord>{};
  records.reserve(frames->size() * 2U);

  auto summaryA = EvalSummary{.label = options->labelA};
  auto summaryB = EvalSummary{.label = options->labelB};

  for (std::size_t index = 0; index < frames->size(); ++index) {
    auto recordA = evaluateOneFrame(index, options->labelA, **modelA, *map, (*frames)[index]);
    auto recordB = evaluateOneFrame(index, options->labelB, **modelB, *map, (*frames)[index]);
    accumulate(summaryA, recordA);
    accumulate(summaryB, recordB);
    records.push_back(std::move(recordA));
    records.push_back(std::move(recordB));
  }

  const auto csvWritten = writeCsv(options->outputCsvPath, records);
  if (!csvWritten) {
    fmt::print(stderr, "[observation_model_debug_app] csv write error: {}\n",
               csvWritten.error().message);
    return 1;
  }

  const auto metricsWritten =
      writeMetricsJson(options->outputJsonPath, summaryA, summaryB, options->configAPath,
                       options->configBPath, frames->size());
  if (!metricsWritten) {
    fmt::print(stderr, "[observation_model_debug_app] metrics write error: {}\n",
               metricsWritten.error().message);
    return 1;
  }

  fmt::print("Observation model evaluation completed.\n");
  fmt::print("  frames: {}\n", frames->size());
  fmt::print("  {} success rate: {:.3f}\n", options->labelA, successRate(summaryA));
  fmt::print("  {} success rate: {:.3f}\n", options->labelB, successRate(summaryB));
  fmt::print("  csv: {}\n", options->outputCsvPath);
  fmt::print("  metrics: {}\n", options->outputJsonPath);
  return 0;
}
