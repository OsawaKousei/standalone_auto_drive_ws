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

class LidarSensor {
public:
  explicit LidarSensor(LidarSensorConfig config);
  [[nodiscard]] auto simulate(const types::MapData &map, const types::Pose &pose) const
      -> Result<types::LidarScan>;

private:
  const LidarSensorConfig config_;
};

} // namespace ad::simulation
