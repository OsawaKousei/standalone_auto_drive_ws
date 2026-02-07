#include "visualizer.hpp"

#include <cmath>
#include <cstdlib>
#include <fmt/core.h>
#include <numbers>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

namespace ad::visualization {

namespace {

[[nodiscard]] auto setEnvValue(const std::string &key, const std::string &value) -> Status {
  if (setenv(key.c_str(), value.c_str(), 1) != 0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = fmt::format("Failed to set {} for matplotlib", key)});
  }
  return {};
}

[[nodiscard]] auto ensurePathContains(const std::string &key, const std::string &value) -> Status {
  auto *const current = std::getenv(key.c_str());
  if (current == nullptr) {
    return setEnvValue(key, value);
  }

  const auto currentValue = std::string{current};
  if (currentValue.find(value) != std::string::npos) {
    return {};
  }

  const auto merged = fmt::format("{}:{}", value, currentValue);
  return setEnvValue(key, merged);
}

[[nodiscard]] auto sitePackagesPath(std::string_view root) -> std::string {
  const auto pythonVersion = fmt::format("{}.{}", PY_MAJOR_VERSION, PY_MINOR_VERSION);
  return fmt::format("{}/lib/python{}/site-packages", root, pythonVersion);
}

} // namespace

auto Visualizer::renderFrame(const types::MapData &map) const -> Status {
  const auto envStatus = configurePythonEnvironment();
  if (!envStatus) {
    return envStatus;
  }

  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Map has invalid dimensions."});
  }

  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  if (map.grid.size() != expectedCells) {
    return tl::make_unexpected(Error{.code = ErrorCode::SizeMismatch,
                                     .message = "Map grid size does not match width and height."});
  }

  last_map_ = MapGeometry{.width = map.width, .height = map.height, .resolution = map.resolution};

  const auto gridImage = [&]() -> std::vector<float> {
    const auto width = static_cast<std::size_t>(map.width);
    const auto height = static_cast<std::size_t>(map.height);
    auto data = std::vector<float>(width * height, 0.0F);
    for (const auto r : std::views::iota(std::size_t{0}, height)) {
      for (const auto c : std::views::iota(std::size_t{0}, width)) {
        const auto idx = (r * width) + c;
        data[idx] = static_cast<float>(map.grid[idx] > 0 ? 1.0 : 0.0);
      }
    }
    return data;
  }();

  const auto extentX = static_cast<double>(map.width);
  const auto extentY = static_cast<double>(map.height);

  matplotlibcpp::clf();
  matplotlibcpp::imshow(gridImage.data(), map.height, map.width, 1, {{"origin", "lower"}});
  matplotlibcpp::xlim(0.0, extentX);
  matplotlibcpp::ylim(0.0, extentY);
  return {};
}

auto Visualizer::renderPath(std::span<const types::Point> path) const -> Status {
  if (path.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Path is empty."});
  }

  const auto map = currentMapGeometry();
  if (!map) {
    return tl::make_unexpected(map.error());
  }

  const auto toCell = [&](double value) -> double { return value / map->resolution; };

  auto xs = std::vector<double>{};
  auto ys = std::vector<double>{};
  xs.reserve(path.size());
  ys.reserve(path.size());
  for (const auto &p : path) {
    xs.push_back(toCell(p.x));
    ys.push_back(toCell(p.y));
  }

  matplotlibcpp::plot(xs, ys, "b-");
  return {};
}

auto Visualizer::renderRobot(const types::Pose &pose, const types::Footprint &footprint) const
    -> Status {
  if (footprint.vertices.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Footprint has no vertices."});
  }

  const auto map = currentMapGeometry();
  if (!map) {
    return tl::make_unexpected(map.error());
  }

  const auto toCell = [&](double value) -> double { return value / map->resolution; };

  auto outlineX = std::vector<double>{};
  auto outlineY = std::vector<double>{};
  outlineX.reserve(footprint.vertices.size() + 1);
  outlineY.reserve(footprint.vertices.size() + 1);

  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);
  for (const auto &vertex : footprint.vertices) {
    const auto gx = pose.x + vertex.x * cosTheta - vertex.y * sinTheta;
    const auto gy = pose.y + vertex.x * sinTheta + vertex.y * cosTheta;
    outlineX.push_back(toCell(gx));
    outlineY.push_back(toCell(gy));
  }

  outlineX.push_back(outlineX.front());
  outlineY.push_back(outlineY.front());

  constexpr double kArrowScale = 0.5;
  const auto hx = toCell(pose.x + std::cos(pose.theta) * kArrowScale);
  const auto hy = toCell(pose.y + std::sin(pose.theta) * kArrowScale);

  matplotlibcpp::plot(outlineX, outlineY, "r-");
  matplotlibcpp::scatter(std::vector<double>{toCell(pose.x)}, std::vector<double>{toCell(pose.y)},
                         40.0, {{"color", "red"}});
  matplotlibcpp::plot({toCell(pose.x), hx}, {toCell(pose.y), hy}, "r-");
  return {};
}

auto Visualizer::renderScan(const types::Pose &pose, std::span<const double> ranges) const
    -> Status {
  if (ranges.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Scan is empty."});
  }

  const auto map = currentMapGeometry();
  if (!map) {
    return tl::make_unexpected(map.error());
  }

  const auto toCell = [&](double value) -> double { return value / map->resolution; };

  auto xs = std::vector<double>{};
  auto ys = std::vector<double>{};
  xs.reserve(ranges.size());
  ys.reserve(ranges.size());
  const auto angleStep = (2.0 * std::numbers::pi) / static_cast<double>(ranges.size());
  for (const auto i : std::views::iota(std::size_t{0}, ranges.size())) {
    const auto angle = pose.theta - std::numbers::pi + (angleStep * static_cast<double>(i));
    const auto distance = ranges[i];
    xs.push_back(toCell(pose.x + (std::cos(angle) * distance)));
    ys.push_back(toCell(pose.y + (std::sin(angle) * distance)));
  }

  matplotlibcpp::scatter(xs, ys, 10.0, {{"color", "green"}});
  matplotlibcpp::pause(0.001);
  matplotlibcpp::show(false);
  return {};
}

auto Visualizer::configurePythonEnvironment() -> Status {
  auto *const pythonHome = std::getenv("PYTHONHOME");
  auto *const virtualEnv = std::getenv("VIRTUAL_ENV");

  if (pythonHome == nullptr && virtualEnv == nullptr) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message =
                                         "Python environment is missing. Set VIRTUAL_ENV or "
                                         "PYTHONHOME to a Python with numpy and matplotlib."});
  }

  const auto pythonRoot = pythonHome != nullptr ? std::string{pythonHome} : std::string{virtualEnv};

  // Do not override PYTHONHOME for embedded interpreter; setting only PYTHONPATH keeps stdlib
  // resolution intact while ensuring site-packages are reachable inside the venv.
  const auto pathStatus = ensurePathContains("PYTHONPATH", sitePackagesPath(pythonRoot));
  if (!pathStatus) {
    return pathStatus;
  }

  return {};
}

auto Visualizer::currentMapGeometry() const -> Result<MapGeometry> {
  if (!last_map_) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Call renderFrame(map) before drawing overlays."});
  }
  return *last_map_;
}

} // namespace ad::visualization
