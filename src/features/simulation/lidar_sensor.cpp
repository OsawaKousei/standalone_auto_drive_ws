#include "lidar_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <random>
#include <ranges>
#include <vector>

namespace {

[[nodiscard]] auto mapHasConsistentGrid(const ad::types::MapData &map) -> bool {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.grid.size() == expectedCells;
}

[[nodiscard]] auto cellIndex(const ad::types::MapData &map, const ad::types::Point &point)
    -> std::optional<std::size_t> {
  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return std::nullopt;
  }

  const auto column = static_cast<int>(std::floor(point.x / map.resolution));
  const auto row = static_cast<int>(std::floor(point.y / map.resolution));

  const bool inBounds = column >= 0 && column < map.width && row >= 0 && row < map.height;
  if (!inBounds) {
    return std::nullopt;
  }

  const auto width = static_cast<std::size_t>(map.width);
  const auto rowIndex = static_cast<std::size_t>(row);
  const auto columnIndex = static_cast<std::size_t>(column);
  const auto index = (rowIndex * width) + columnIndex;
  return index;
}

} // namespace

namespace ad::simulation {

LidarSensor::LidarSensor(LidarSensorConfig config)
    : config_(config), generator_(std::random_device{}()) {}

auto LidarSensor::simulate(const types::MapData &map, const types::Pose &pose) const
    -> Result<types::LidarScan> {
  if (!mapHasConsistentGrid(map)) {
    return tl::make_unexpected(Error{.code = ErrorCode::SizeMismatch,
                                     .message = "Map grid size does not match width and height."});
  }

  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Map dimensions must be positive."});
  }

  const auto rayCountRaw = config_.rayCount > 0 ? config_.rayCount : std::max(map.width, 1);
  const auto rayCount = static_cast<std::size_t>(rayCountRaw);
  const auto maxRange = config_.maxRange > 0.0
                            ? config_.maxRange
                            : map.resolution * static_cast<double>(std::max(map.width, map.height));
  const auto rangeStep = config_.rangeStep > 0.0 ? config_.rangeStep : map.resolution;
  if (config_.maxAngle <= config_.minAngle) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Lidar angle range is invalid."});
  }
  if (config_.rangeNoiseStddev < 0.0 || !std::isfinite(config_.rangeNoiseStddev)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Lidar noise stddev is invalid."});
  }
  const auto angleStep =
      rayCount > 1U ? (config_.maxAngle - config_.minAngle) / static_cast<double>(rayCount - 1U)
                    : 0.0;
  const auto stepLimit = static_cast<std::size_t>(std::ceil(maxRange / rangeStep));

  const auto sampleNoise = [&]() {
    if (config_.rangeNoiseStddev == 0.0) {
      return 0.0;
    }
    auto distribution = std::normal_distribution<double>{0.0, config_.rangeNoiseStddev};
    return distribution(generator_);
  };

  const auto traceRay = [&](double angle) -> double {
    const auto steps = std::views::iota(std::size_t{1}, stepLimit + 1);
    for (const auto stepIndex : steps) {
      const auto distance = rangeStep * static_cast<double>(stepIndex);
      const auto xValue = pose.x + (std::cos(angle) * distance);
      const auto yValue = pose.y + (std::sin(angle) * distance);
      const auto index = cellIndex(map, types::Point{.x = xValue, .y = yValue});
      if (!index.has_value()) {
        return maxRange;
      }
      if (map.grid[*index] > 0) {
        return std::clamp(distance, 0.0, maxRange);
      }
    }

    return maxRange;
  };

  auto ranges = std::vector<double>{};
  ranges.reserve(rayCount);

  const auto rayIndices = std::views::iota(std::size_t{0}, rayCount);
  std::ranges::transform(rayIndices, std::back_inserter(ranges), [&](std::size_t index) -> double {
    const auto angle = pose.theta + config_.minAngle + (angleStep * static_cast<double>(index));
    const auto noiselessRange = traceRay(angle);
    const auto noisyRange = noiselessRange + sampleNoise();
    return std::clamp(noisyRange, 0.0, maxRange);
  });

  return types::LidarScan{.ranges = std::move(ranges),
                          .minAngle = config_.minAngle,
                          .angleIncrement = angleStep,
                          .maxRange = maxRange};
}

} // namespace ad::simulation
