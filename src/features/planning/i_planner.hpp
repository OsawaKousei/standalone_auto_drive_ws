#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

namespace ad::planning {

class IPlanner {
public:
  virtual ~IPlanner() = default;
  IPlanner() = default;
  IPlanner(const IPlanner &) = delete;
  auto operator=(const IPlanner &) -> IPlanner = delete;
  IPlanner(IPlanner &&) = delete;
  auto operator=(IPlanner &&) -> IPlanner = delete;
  [[nodiscard]] virtual auto plan(const types::MapData &map, const types::Pose &start,
                                  const types::Pose &goal) const -> Result<types::Path> = 0;
};

} // namespace ad::planning
