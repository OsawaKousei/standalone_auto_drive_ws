#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

namespace ad::simulation {

using LidarScan = types::LidarScan;

class ILidarSensor {
public:
  virtual ~ILidarSensor() = default;
  ILidarSensor() = default;
  ILidarSensor(const ILidarSensor &) = delete;
  auto operator=(const ILidarSensor &) -> ILidarSensor & = delete;
  ILidarSensor(ILidarSensor &&) = delete;
  auto operator=(ILidarSensor &&) -> ILidarSensor & = delete;
  [[nodiscard]] virtual auto simulate(const types::MapData &map, const types::Pose &pose) const
      -> Result<LidarScan> = 0;
};

} // namespace ad::simulation
