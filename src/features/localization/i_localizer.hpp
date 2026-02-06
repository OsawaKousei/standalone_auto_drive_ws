#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <span>

namespace ad::localization {

class ILocalizer {
public:
  virtual ~ILocalizer() = default;
  [[nodiscard]] virtual Result<types::Pose> estimate(std::span<const double> scan,
                                                     const types::MapData &map,
                                                     const types::Pose &initialPose) const = 0;
};

} // namespace ad::localization
