#include "visualizer.hpp"

#include <algorithm>
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
        Error{ErrorCode::InvalidInput, fmt::format("Failed to set {} for matplotlib", key)});
  }
  return {};
}

[[nodiscard]] auto ensurePathContains(const std::string &key, const std::string &value) -> Status {
  const auto current = std::getenv(key.c_str());
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

auto Visualizer::renderPath(std::span<const types::Point> path) const -> Status {
  if (path.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Path is empty."});
  }
  fmt::print("Render path with {} waypoints\n", path.size());
  return {};
}

auto Visualizer::renderPose(const types::Pose &pose) const -> Status {
  fmt::print("Render pose -> x: {:.2f}, y: {:.2f}, theta: {:.2f}\n", pose.x, pose.y, pose.theta);
  return {};
}

auto Visualizer::renderScan(std::span<const double> ranges) const -> Status {
  if (ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan is empty."});
  }

  const auto [minIt, maxIt] = std::ranges::minmax_element(ranges);
  fmt::print("Render scan ({} rays) -> min: {:.2f} m, max: {:.2f} m\n", ranges.size(), *minIt,
             *maxIt);
  return {};
}

auto Visualizer::renderFrame(const types::MapData &map, const types::Pose &pose,
                             std::span<const types::Point> path,
                             std::span<const double> ranges) const -> Status {
  const auto envStatus = configurePythonEnvironment();
  if (!envStatus) {
    return envStatus;
  }

  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Invalid map geometry."});
  }

  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  if (map.grid.size() != expectedCells) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan is empty."});
  }

  const auto gridImage = [&]() {
    auto data = std::vector<float>(static_cast<std::size_t>(map.width * map.height), 0.0F);
    for (int r = 0; r < map.height; ++r) {
      for (int c = 0; c < map.width; ++c) {
        const auto idx = static_cast<std::size_t>(r * map.width + c);
        data[idx] = static_cast<float>(map.grid[idx] > 0 ? 1.0 : 0.0);
      }
    }
    return data;
  }();

  const auto toCell = [&](double value) { return value / map.resolution; };

  const auto makePathXY = [&]() {
    auto xs = std::vector<double>{};
    auto ys = std::vector<double>{};
    xs.reserve(path.size());
    ys.reserve(path.size());
    for (const auto &p : path) {
      xs.push_back(toCell(p.x));
      ys.push_back(toCell(p.y));
    }
    return std::pair{xs, ys};
  }();

  const auto scanPoints = [&]() {
    auto xs = std::vector<double>{};
    auto ys = std::vector<double>{};
    xs.reserve(ranges.size());
    ys.reserve(ranges.size());
    const auto angleStep = (2.0 * std::numbers::pi) / static_cast<double>(ranges.size());
    for (const auto i : std::views::iota(std::size_t{0}, ranges.size())) {
      const auto angle = pose.theta - std::numbers::pi + angleStep * static_cast<double>(i);
      const auto distance = ranges[i];
      xs.push_back(toCell(pose.x + std::cos(angle) * distance));
      ys.push_back(toCell(pose.y + std::sin(angle) * distance));
    }
    return std::pair{xs, ys};
  }();

  const auto heading = [&]() {
    constexpr double kArrowScale = 0.5;
    const auto hx = toCell(pose.x + std::cos(pose.theta) * kArrowScale);
    const auto hy = toCell(pose.y + std::sin(pose.theta) * kArrowScale);
    return std::pair{hx, hy};
  }();

  const auto extentX = static_cast<double>(map.width);
  const auto extentY = static_cast<double>(map.height);

  matplotlibcpp::clf();
  matplotlibcpp::imshow(gridImage.data(), map.height, map.width, 1, {{"origin", "lower"}});
  matplotlibcpp::xlim(0.0, extentX);
  matplotlibcpp::ylim(0.0, extentY);

  if (!path.empty()) {
    matplotlibcpp::plot(makePathXY.first, makePathXY.second, "b-");
  }

  matplotlibcpp::scatter(std::vector<double>{toCell(pose.x)}, std::vector<double>{toCell(pose.y)},
                         40.0, {{"color", "red"}});
  matplotlibcpp::plot({toCell(pose.x), heading.first}, {toCell(pose.y), heading.second}, "r-");

  matplotlibcpp::scatter(scanPoints.first, scanPoints.second, 10.0, {{"color", "green"}});
  matplotlibcpp::pause(0.001);
  matplotlibcpp::show(false);
  return {};
}

auto Visualizer::configurePythonEnvironment() -> Status {
  const auto pythonHome = std::getenv("PYTHONHOME");
  const auto virtualEnv = std::getenv("VIRTUAL_ENV");

  if (pythonHome == nullptr && virtualEnv == nullptr) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput,
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

} // namespace ad::visualization
