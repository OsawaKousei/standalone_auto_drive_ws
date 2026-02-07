#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <vector>

namespace ad::simulation {

using LidarScan = std::vector<double>;

class ISensorModel {
public:
  virtual ~ISensorModel() = default;
  ISensorModel() = default;
  ISensorModel(const ISensorModel &) = delete;
  auto operator=(const ISensorModel &) -> ISensorModel = delete;
  ISensorModel(ISensorModel &&) = delete;
  auto operator=(ISensorModel &&) -> ISensorModel = delete;
  [[nodiscard]] virtual auto simulate(const types::MapData &map, const types::Pose &pose) const
      -> Result<LidarScan> = 0;
};

} // namespace ad::simulation
