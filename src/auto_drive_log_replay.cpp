#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <chrono>
#include <cstddef>
#include <fstream>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <fmt/core.h>

namespace ad::demo {

struct LogRecord {
  int step = 0;
  types::Pose truePose{};
  types::Pose estPose{};
  std::vector<types::Point> scanPoints;
  std::vector<double> ranges;
};

struct ScanMeta {
  double minAngle = 0.0;
  double angleIncrement = 0.0;
  double maxRange = 0.0;
  std::size_t count = 0;
};

struct LogData {
  std::vector<LogRecord> records;
  types::Footprint footprint;
  std::vector<types::Point> path;
  std::optional<ScanMeta> scanMeta;
};

struct LogColumns {
  std::size_t step = 0;
  std::size_t trueX = 0;
  std::size_t trueY = 0;
  std::size_t trueTheta = 0;
  std::size_t estX = 0;
  std::size_t estY = 0;
  std::size_t estTheta = 0;
  std::optional<std::size_t> scanPoints{};
};

[[nodiscard]] auto makeFootprint() -> types::Footprint {
  return types::Footprint{{{-0.2, -0.1}, {0.3, -0.1}, {0.3, 0.1}, {-0.2, 0.1}}};
}

[[nodiscard]] auto splitCsvLine(std::string_view line) -> std::vector<std::string> {
  auto output = std::vector<std::string>{};
  std::stringstream string_stream{std::string{line}};
  for (std::string cell; std::getline(string_stream, cell, ',');) {
    output.push_back(cell);
  }
  return output;
}

[[nodiscard]] auto splitDelimited(std::string_view input, char delimiter)
    -> std::vector<std::string> {
  auto output = std::vector<std::string>{};
  std::stringstream string_stream{std::string{input}};
  for (std::string cell; std::getline(string_stream, cell, delimiter);) {
    if (!cell.empty()) {
      output.push_back(cell);
    }
  }
  return output;
}

[[nodiscard]] auto parsePoints(std::string_view input) -> Result<std::vector<types::Point>> {
  auto points = std::vector<types::Point>{};
  if (input.empty()) {
    return points;
  }

  for (const auto &token : splitDelimited(input, ';')) {
    const auto pair = splitDelimited(token, ':');
    if (pair.size() != 2U) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse point list."});
    }
    try {
      points.push_back(types::Point{.x = std::stod(pair[0]), .y = std::stod(pair[1])});
    } catch (const std::exception &) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse point values."});
    }
  }

  return points;
}

[[nodiscard]] auto parseScanMeta(std::string_view input) -> Result<ScanMeta> {
  const auto parts = splitDelimited(input, ';');
  if (parts.size() != 4U) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse scan meta."});
  }

  try {
    return ScanMeta{.minAngle = std::stod(parts[0]),
                    .angleIncrement = std::stod(parts[1]),
                    .maxRange = std::stod(parts[2]),
                    .count = static_cast<std::size_t>(std::stoul(parts[3]))};
  } catch (const std::exception &) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse scan meta values."});
  }
}

[[nodiscard]] auto parseLogColumns(std::string_view header) -> Result<LogColumns> {
  const auto names = splitCsvLine(header);
  auto indexByName = std::unordered_map<std::string, std::size_t>{};
  indexByName.reserve(names.size());
  for (const auto index : std::views::iota(std::size_t{0}, names.size())) {
    indexByName.emplace(names[index], index);
  }

  const auto getRequiredIndex = [&](const std::string &name) -> Result<std::size_t> {
    const auto iterator = indexByName.find(name);
    if (iterator == indexByName.end()) {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Missing required log column: " + name});
    }
    return iterator->second;
  };

  const auto stepIndex = getRequiredIndex("step");
  if (!stepIndex) {
    return tl::make_unexpected(stepIndex.error());
  }
  const auto trueXIndex = getRequiredIndex("true_x");
  if (!trueXIndex) {
    return tl::make_unexpected(trueXIndex.error());
  }
  const auto trueYIndex = getRequiredIndex("true_y");
  if (!trueYIndex) {
    return tl::make_unexpected(trueYIndex.error());
  }
  const auto trueThetaIndex = getRequiredIndex("true_theta");
  if (!trueThetaIndex) {
    return tl::make_unexpected(trueThetaIndex.error());
  }
  const auto estXIndex = getRequiredIndex("est_x");
  if (!estXIndex) {
    return tl::make_unexpected(estXIndex.error());
  }
  const auto estYIndex = getRequiredIndex("est_y");
  if (!estYIndex) {
    return tl::make_unexpected(estYIndex.error());
  }
  const auto estThetaIndex = getRequiredIndex("est_theta");
  if (!estThetaIndex) {
    return tl::make_unexpected(estThetaIndex.error());
  }

  auto scanPointsIndex = std::optional<std::size_t>{};
  if (const auto iterator = indexByName.find("scan_points"); iterator != indexByName.end()) {
    scanPointsIndex.emplace(iterator->second);
  }

  return LogColumns{.step = *stepIndex,
                    .trueX = *trueXIndex,
                    .trueY = *trueYIndex,
                    .trueTheta = *trueThetaIndex,
                    .estX = *estXIndex,
                    .estY = *estYIndex,
                    .estTheta = *estThetaIndex,
                    .scanPoints = scanPointsIndex};
}

[[nodiscard]] auto parseLogFile(const std::string &path) -> Result<LogData> {
  auto file = std::ifstream{path};
  if (!file.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to open log file."});
  }

  auto records = std::vector<LogRecord>{};
  auto footprint = std::optional<types::Footprint>{};
  auto pathPoints = std::optional<std::vector<types::Point>>{};
  auto scanMeta = std::optional<ScanMeta>{};
  auto columns = std::optional<LogColumns>{};
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line.starts_with('#') || line.starts_with("step,")) {
      if (line.starts_with("# footprint=")) {
        const auto payload = std::string_view{line}.substr(std::string_view{"# footprint="}.size());
        const auto parsed = parsePoints(payload);
        if (!parsed) {
          return tl::make_unexpected(parsed.error());
        }
        footprint.emplace(types::Footprint{*parsed});
      }
      if (line.starts_with("# path=")) {
        const auto payload = std::string_view{line}.substr(std::string_view{"# path="}.size());
        const auto parsed = parsePoints(payload);
        if (!parsed) {
          return tl::make_unexpected(parsed.error());
        }
        pathPoints.emplace(*parsed);
      }
      if (line.starts_with("# scan_meta=")) {
        const auto payload = std::string_view{line}.substr(std::string_view{"# scan_meta="}.size());
        const auto parsed = parseScanMeta(payload);
        if (!parsed) {
          return tl::make_unexpected(parsed.error());
        }
        scanMeta = *parsed;
      }
      if (line.starts_with("step,")) {
        const auto parsed = parseLogColumns(line);
        if (!parsed) {
          return tl::make_unexpected(parsed.error());
        }
        columns = *parsed;
      }
      continue;
    }

    const auto cells = splitCsvLine(line);
    if (!columns) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput, .message = "Missing log header row."});
    }

    const auto maxRequiredIndex =
        std::max({columns->step, columns->trueX, columns->trueY, columns->trueTheta, columns->estX,
                  columns->estY, columns->estTheta});
    if (cells.size() <= maxRequiredIndex) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::SizeMismatch, .message = "Unexpected column count."});
    }

    try {
      const auto step = std::stoi(cells[columns->step]);
      const auto truePose = types::Pose{.x = std::stod(cells[columns->trueX]),
                                        .y = std::stod(cells[columns->trueY]),
                                        .theta = std::stod(cells[columns->trueTheta])};
      const auto estPose = types::Pose{.x = std::stod(cells[columns->estX]),
                                       .y = std::stod(cells[columns->estY]),
                                       .theta = std::stod(cells[columns->estTheta])};
      auto scanPoints = std::vector<types::Point>{};
      auto ranges = std::vector<double>{};
      if (columns->scanPoints.has_value() && cells.size() > *columns->scanPoints) {
        const auto &scanCell = cells[*columns->scanPoints];
        if (scanCell.find(':') != std::string::npos) {
          auto parsed = parsePoints(scanCell);
          if (!parsed) {
            return tl::make_unexpected(parsed.error());
          }
          scanPoints = std::move(*parsed);
        } else {
          for (const auto &value : splitDelimited(scanCell, ';')) {
            if (value.empty()) {
              continue;
            }
            ranges.push_back(std::stod(value));
          }
        }
      }
      if (scanMeta && !ranges.empty() && scanMeta->count != ranges.size()) {
        return tl::make_unexpected(
            Error{.code = ErrorCode::SizeMismatch, .message = "Scan range count mismatch."});
      }
      records.push_back(LogRecord{.step = step,
                                  .truePose = truePose,
                                  .estPose = estPose,
                                  .scanPoints = std::move(scanPoints),
                                  .ranges = std::move(ranges)});
    } catch (const std::exception &) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse log row."});
    }
  }

  if (records.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Log has no records."});
  }

  const auto footprintValue = footprint ? *footprint : makeFootprint();
  const auto pathValue = pathPoints ? *pathPoints : std::vector<types::Point>{};

  return LogData{.records = std::move(records),
                 .footprint = footprintValue,
                 .path = pathValue,
                 .scanMeta = scanMeta};
}

struct Args {
  std::string logPath = "logs/localization_control_lidar_demo.log";
  std::string mapPath = "tools/map.yaml";
  int delayMs = 80;
};

[[nodiscard]] auto parseArgs(int argc, char **argv) -> Result<Args> {
  auto args = Args{};
  for (int index = 1; index < argc; ++index) {
    const std::string_view token{argv[index]};
    if (token == "--log" && index + 1 < argc) {
      args.logPath = argv[++index];
      continue;
    }
    if (token == "--map" && index + 1 < argc) {
      args.mapPath = argv[++index];
      continue;
    }
    if (token == "--delay-ms" && index + 1 < argc) {
      try {
        args.delayMs = std::stoi(argv[++index]);
      } catch (const std::exception &) {
        return tl::make_unexpected(
            Error{.code = ErrorCode::InvalidInput, .message = "delay-ms must be an integer."});
      }
      continue;
    }
    if (token == "--help") {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput,
                .message = "Usage: localization_control_lidar_log_replay [--log PATH] [--map PATH] "
                           "[--delay-ms N]"});
    }

    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Unknown argument."});
  }

  if (args.delayMs < 0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "delay-ms must be non-negative."});
  }

  return args;
}

} // namespace ad::demo

auto main(int argc, char **argv) -> int {
  const auto argsResult = ad::demo::parseArgs(argc, argv);
  if (!argsResult) {
    fmt::print(stderr, "{}\n", argsResult.error().message);
    return 1;
  }
  const auto args = *argsResult;

  const auto mapResult = ad::loadMapFromYaml(args.mapPath);
  if (!mapResult) {
    fmt::print(stderr, "Map load error: {}\n", mapResult.error().message);
    return 1;
  }

  const auto logDataResult = ad::demo::parseLogFile(args.logPath);
  if (!logDataResult) {
    fmt::print(stderr, "Log parse error: {}\n", logDataResult.error().message);
    return 1;
  }
  const auto &logData = *logDataResult;

  const auto &footprint = logData.footprint;
  const ad::visualization::Visualizer viz;
  const auto preparedMapResult = ad::visualization::Visualizer::prepareMap(*mapResult);
  if (!preparedMapResult) {
    fmt::print(stderr, "Render error: {}\n", preparedMapResult.error().message);
    return 1;
  }
  const auto &preparedMap = *preparedMapResult;
  const auto &mapGeometry = preparedMap.geometry;

  auto trueTrail = std::vector<ad::types::Point>{};
  auto estTrail = std::vector<ad::types::Point>{};
  trueTrail.reserve(logData.records.size());
  estTrail.reserve(logData.records.size());

  for (const auto &record : logData.records) {
    const auto frameStatus = viz.renderFrame(preparedMap);
    if (!frameStatus) {
      fmt::print(stderr, "Render error: {}\n", frameStatus.error().message);
      return 1;
    }

    if (!logData.path.empty()) {
      const auto plannedPathStatus = viz.renderPath(std::span{logData.path}, mapGeometry, "k--");
      if (!plannedPathStatus) {
        fmt::print(stderr, "Render error: {}\n", plannedPathStatus.error().message);
        return 1;
      }
    }

    trueTrail.push_back(ad::types::Point{.x = record.truePose.x, .y = record.truePose.y});
    estTrail.push_back(ad::types::Point{.x = record.estPose.x, .y = record.estPose.y});

    const auto truePathStatus = viz.renderPath(std::span{trueTrail}, mapGeometry);
    if (!truePathStatus) {
      fmt::print(stderr, "Render error: {}\n", truePathStatus.error().message);
      return 1;
    }

    const auto estPathStatus = viz.renderPath(std::span{estTrail}, mapGeometry, "c--");
    if (!estPathStatus) {
      fmt::print(stderr, "Render error: {}\n", estPathStatus.error().message);
      return 1;
    }

    const auto robotStatus = viz.renderRobot(record.truePose, footprint, mapGeometry);
    if (!robotStatus) {
      fmt::print(stderr, "Render error: {}\n", robotStatus.error().message);
      return 1;
    }

    if (!record.scanPoints.empty()) {
      const auto scanStatus =
          viz.renderPoints(std::span{record.scanPoints}, mapGeometry, 10.0, "green");
      if (!scanStatus) {
        fmt::print(stderr, "Render error: {}\n", scanStatus.error().message);
        return 1;
      }
    } else if (logData.scanMeta && !record.ranges.empty()) {
      const auto scan = ad::types::LidarScan{.ranges = record.ranges,
                                             .minAngle = logData.scanMeta->minAngle,
                                             .angleIncrement = logData.scanMeta->angleIncrement,
                                             .maxRange = logData.scanMeta->maxRange};
      const auto scanStatus = viz.renderScan(record.truePose, scan, mapGeometry);
      if (!scanStatus) {
        fmt::print(stderr, "Render error: {}\n", scanStatus.error().message);
        return 1;
      }
    }

    const auto estMarkerStatus = viz.renderMarker(
        ad::types::Point{.x = record.estPose.x, .y = record.estPose.y}, mapGeometry, 0.15, "cyan");
    if (!estMarkerStatus) {
      fmt::print(stderr, "Render error: {}\n", estMarkerStatus.error().message);
      return 1;
    }

    const auto presentStatus = viz.presentFrame();
    if (!presentStatus) {
      fmt::print(stderr, "Render error: {}\n", presentStatus.error().message);
      return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{args.delayMs});
  }

  const auto saveStatus = viz.saveFigure("localization_control_lidar_log_replay.png");
  if (!saveStatus) {
    fmt::print(stderr, "Render error: {}\n", saveStatus.error().message);
    return 1;
  }

  return 0;
}
