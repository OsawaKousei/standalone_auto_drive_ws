#pragma once

#include "i_sensor.hpp"

namespace ad::simulation {

struct LidarSimConfig {
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

[[nodiscard]] inline auto lidarDefaultConfig() -> LidarSimConfig {
  return LidarSimConfig{.rayCount = kDefaultRayCount,
                        .minAngle = kDefaultMinAngle,
                        .maxAngle = kDefaultMaxAngle,
                        .maxRange = kDefaultMaxRange,
                        .rangeStep = kDefaultRangeStep};
}

} // namespace config

class LidarSim final : public ILidarSensor {
public:
  explicit LidarSim(LidarSimConfig config = config::lidarDefaultConfig());
  [[nodiscard]] auto simulate(const types::MapData &map, const types::Pose &pose) const
      -> Result<LidarScan> override;

private:
  const LidarSimConfig config_;
};

} // namespace ad::simulation
