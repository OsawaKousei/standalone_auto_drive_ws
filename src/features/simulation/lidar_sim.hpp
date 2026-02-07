#pragma once

#include "i_sensor.hpp"

namespace ad::simulation {

class LidarSim final : public ISensorModel {
public:
  LidarSim() = default;
  [[nodiscard]] auto simulate(const types::MapData &map, const types::Pose &pose) const
      -> Result<LidarScan> override;
};

} // namespace ad::simulation
