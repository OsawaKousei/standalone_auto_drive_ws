#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

namespace ad::planning {

class ICollisionChecker {
public:
  virtual ~ICollisionChecker() = default;
  ICollisionChecker() = default;
  ICollisionChecker(const ICollisionChecker &) = delete;
  auto operator=(const ICollisionChecker &) -> ICollisionChecker = delete;
  ICollisionChecker(ICollisionChecker &&) = delete;
  auto operator=(ICollisionChecker &&) -> ICollisionChecker = delete;

  [[nodiscard]] virtual auto isFree(const types::Pose &pose,
                                    const types::Footprint &footprint) const -> Result<bool> = 0;
};

} // namespace ad::planning
