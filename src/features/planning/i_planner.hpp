#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

namespace ad::planning {

class IPlanner {
public:
  virtual ~IPlanner() = default;
  [[nodiscard]] virtual Result<types::Path>
  plan(const types::MapData &map, const types::Pose &start, const types::Pose &goal) const = 0;
};

} // namespace ad::planning
