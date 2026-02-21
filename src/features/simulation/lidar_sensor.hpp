#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

namespace ad::simulation {

struct LidarSensorConfig {
  const int rayCount;
  const double minAngle;
  const double maxAngle;
  const double maxRange;
  const double rangeStep;
};

namespace config {

constexpr int kDefaultRayCount = 0;
constexpr double kDefaultMinAngle = -3.14159265358979323846;
constexpr double kDefaultMaxAngle = 3.14159265358979323846;
constexpr double kDefaultMaxRange = 0.0;
constexpr double kDefaultRangeStep = 0.0;

[[nodiscard]] inline auto lidarDefaultConfig() -> LidarSensorConfig {
  return LidarSensorConfig{.rayCount = kDefaultRayCount,
                           .minAngle = kDefaultMinAngle,
                           .maxAngle = kDefaultMaxAngle,
                           .maxRange = kDefaultMaxRange,
                           .rangeStep = kDefaultRangeStep};
}

} // namespace config

class LidarSensor {
public:
  explicit LidarSensor(LidarSensorConfig config = config::lidarDefaultConfig());
  [[nodiscard]] auto simulate(const types::MapData &map, const types::Pose &pose) const
      -> Result<types::LidarScan>;

private:
  const LidarSensorConfig config_;
};

} // namespace ad::simulation
