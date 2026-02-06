#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <vector>

namespace ad::simulation {

using LidarScan = std::vector<double>;

class ISensorModel {
public:
  virtual ~ISensorModel() = default;
  [[nodiscard]] virtual Result<LidarScan> simulate(const types::MapData &map,
                                                   const types::Pose &pose) const = 0;
};

} // namespace ad::simulation
