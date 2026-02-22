#include "features/localization/observation_model/line_extractor.hpp"
#include "features/visualization/visualizer.hpp"
#include "shared/map_loader.hpp"
#include "shared/result.hpp"
#include "shared/text_config.hpp"

#include <filesystem>
#include <fmt/core.h>
#include <string>
#include <string_view>
#include <vector>

namespace ad::line_extractor_overlay_test {

constexpr double kEndpointMarkerSize = 30.0;

struct ProgramOptions {
  std::string scenarioPath = "configs/scenario.toml";
  std::string outputPath = "test/localization/logs/line_extractor_overlay.png";
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

[[nodiscard]] auto resolvePath(std::string_view baseDir, std::string_view path) -> std::string {
  const auto candidate = std::filesystem::path{std::string{path}};
  if (candidate.is_absolute()) {
    return candidate.lexically_normal().string();
  }

  return (std::filesystem::path{std::string{baseDir}} / candidate).lexically_normal().string();
}

[[nodiscard]] auto parseArgs(int argc, char **argv) -> Result<ProgramOptions> {
  auto options = ProgramOptions{};
  for (int index = 1; index < argc; ++index) {
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

    if (arg == "--scenario") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.scenarioPath = *value;
    } else if (arg == "--output") {
      auto value = readValue(arg);
      if (!value) {
        return tl::make_unexpected(value.error());
      }
      options.outputPath = *value;
    } else {
      return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                       .message = "Unknown argument: " + std::string{arg}});
    }
  }

  return options;
}

auto run(const ProgramOptions &options) -> Result<int> {
  const auto scenarioPath = std::filesystem::path{options.scenarioPath}.lexically_normal();
  const auto scenarioCfg = config::loadTextConfig(scenarioPath.string());
  if (!scenarioCfg) {
    return tl::make_unexpected(scenarioCfg.error());
  }

  const auto scenarioDir =
      scenarioPath.parent_path().empty() ? std::filesystem::path{"."} : scenarioPath.parent_path();

  const auto mapYamlRaw = requiredString(*scenarioCfg, "map", "yaml_path");
  if (!mapYamlRaw) {
    return tl::make_unexpected(mapYamlRaw.error());
  }
  const auto mapYamlPath = resolvePath(scenarioDir.string(), *mapYamlRaw);

  const auto map = loadMapFromYaml(mapYamlPath);
  if (!map) {
    return tl::make_unexpected(map.error());
  }

  const auto lines = localization::line_extractor::extractMapLinesFromMap(*map);
  if (!lines) {
    return tl::make_unexpected(lines.error());
  }

  const auto prepared = visualization::Visualizer::prepareMap(*map);
  if (!prepared) {
    return tl::make_unexpected(prepared.error());
  }

  auto visualizer = visualization::Visualizer{};
  if (const auto status = visualizer.renderFrame(*prepared); !status) {
    return tl::make_unexpected(status.error());
  }

  for (const auto &line : *lines) {
    const auto path = std::vector<types::Point>{line.segment.start, line.segment.end};
    if (const auto status = visualizer.renderPath(path, prepared->geometry, "m-"); !status) {
      return tl::make_unexpected(status.error());
    }

    if (const auto status = visualizer.renderMarker(line.segment.start, prepared->geometry,
                                                    kEndpointMarkerSize, "lime");
        !status) {
      return tl::make_unexpected(status.error());
    }
    if (const auto status = visualizer.renderMarker(line.segment.end, prepared->geometry,
                                                    kEndpointMarkerSize, "cyan");
        !status) {
      return tl::make_unexpected(status.error());
    }
  }

  const auto outputPath = std::filesystem::path{options.outputPath}.lexically_normal();
  if (!outputPath.parent_path().empty()) {
    std::error_code error;
    std::filesystem::create_directories(outputPath.parent_path(), error);
  }

  if (const auto status = visualizer.saveFigure(outputPath.string()); !status) {
    return tl::make_unexpected(status.error());
  }

  fmt::print("Saved overlay image: {}\n", outputPath.string());
  fmt::print("Extracted line segments: {}\n", lines->size());
  return 0;
}

} // namespace ad::line_extractor_overlay_test

auto main(int argc, char **argv) -> int {
  const auto options = ad::line_extractor_overlay_test::parseArgs(argc, argv);
  if (!options) {
    fmt::print(stderr, "line_extractor_overlay_test_app argument error: {}\n",
               options.error().message);
    fmt::print(stderr, "Usage: {} [--scenario <path>] [--output <path>]\n",
               argc > 0 ? argv[0] : "line_extractor_overlay_test_app");
    return 1;
  }

  const auto result = ad::line_extractor_overlay_test::run(*options);
  if (!result) {
    fmt::print(stderr, "line_extractor_overlay_test_app failed: {}\n", result.error().message);
    return 1;
  }

  return *result;
}
