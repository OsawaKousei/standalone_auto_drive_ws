#include "visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/core.h>
#include <numbers>
#include <ranges>
#include <vector>

namespace ad::visualization {

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
    auto rows = std::vector<std::vector<double>>(
        static_cast<std::size_t>(map.height),
        std::vector<double>(static_cast<std::size_t>(map.width), 0.0));
    for (int r = 0; r < map.height; ++r) {
      for (int c = 0; c < map.width; ++c) {
        const auto idx = static_cast<std::size_t>(r * map.width + c);
        rows[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] =
            static_cast<double>(map.grid[idx] > 0 ? 1.0 : 0.0);
      }
    }
    return rows;
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
  matplotlibcpp::imshow(gridImage, {{"origin", "lower"}});
  matplotlibcpp::xlim(0.0, extentX);
  matplotlibcpp::ylim(0.0, extentY);

  if (!path.empty()) {
    matplotlibcpp::plot(makePathXY.first, makePathXY.second, "b-");
  }

  matplotlibcpp::scatter({toCell(pose.x)}, {toCell(pose.y)}, 40.0, {{"color", "red"}});
  matplotlibcpp::plot({toCell(pose.x), heading.first}, {toCell(pose.y), heading.second}, "r-");

  matplotlibcpp::scatter(scanPoints.first, scanPoints.second, 10.0, {{"color", "green"}});
  matplotlibcpp::pause(0.001);
  matplotlibcpp::show(false);
  return {};
}

} // namespace ad::visualization
