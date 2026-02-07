#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/types.hpp"

#include <chrono>
#include <cstddef>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <fmt/core.h>

namespace ad::demo {

struct LogRecord {
  int step = 0;
  types::Pose truePose{};
  types::Pose estPose{};
};

[[nodiscard]] auto makeFootprint() -> types::Footprint {
  return types::Footprint{{{-0.2, -0.1}, {0.3, -0.1}, {0.3, 0.1}, {-0.2, 0.1}}};
}

[[nodiscard]] auto splitCsvLine(std::string_view line) -> std::vector<std::string> {
  auto output = std::vector<std::string>{};
  std::stringstream ss{std::string{line}};
  for (std::string cell; std::getline(ss, cell, ',');) {
    output.push_back(cell);
  }
  return output;
}

[[nodiscard]] auto parseLogFile(const std::string &path) -> Result<std::vector<LogRecord>> {
  auto file = std::ifstream{path};
  if (!file.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to open log file."});
  }

  auto records = std::vector<LogRecord>{};
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line.starts_with('#') || line.starts_with("step,")) {
      continue;
    }

    const auto cells = splitCsvLine(line);
    if (cells.size() != 14U) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::SizeMismatch, .message = "Unexpected column count."});
    }

    try {
      const auto step = std::stoi(cells[0]);
      const auto truePose =
          types::Pose{std::stod(cells[6]), std::stod(cells[7]), std::stod(cells[8])};
      const auto estPose =
          types::Pose{std::stod(cells[9]), std::stod(cells[10]), std::stod(cells[11])};
      records.push_back(LogRecord{.step = step, .truePose = truePose, .estPose = estPose});
    } catch (const std::exception &) {
      return tl::make_unexpected(
          Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse log row."});
    }
  }

  if (records.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Log has no records."});
  }

  return records;
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

  const auto logResult = ad::demo::parseLogFile(args.logPath);
  if (!logResult) {
    fmt::print(stderr, "Log parse error: {}\n", logResult.error().message);
    return 1;
  }

  const auto footprint = ad::demo::makeFootprint();
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
  trueTrail.reserve(logResult->size());
  estTrail.reserve(logResult->size());

  for (const auto &record : *logResult) {
    const auto frameStatus = viz.renderFrame(preparedMap);
    if (!frameStatus) {
      fmt::print(stderr, "Render error: {}\n", frameStatus.error().message);
      return 1;
    }

    trueTrail.push_back(ad::types::Point{record.truePose.x, record.truePose.y});
    estTrail.push_back(ad::types::Point{record.estPose.x, record.estPose.y});

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

    const auto estMarkerStatus = viz.renderMarker(
        ad::types::Point{record.estPose.x, record.estPose.y}, mapGeometry, 20.0, "cyan");
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
